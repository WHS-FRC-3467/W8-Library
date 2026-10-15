/*
 * Copyright (C) 2026 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */
package frc.lib.devices;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import frc.lib.io.vision.C2Decoder;
import frc.lib.io.vision.PhotonDecoder;
import frc.lib.io.vision.VisionIO;
import frc.lib.io.vision.VisionIO.CameraResult;
import frc.lib.io.vision.VisionIO.NativePacketType;
import frc.lib.io.vision.VisionIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

/**
 * A group of cameras whose results arrive through one {@link VisionIO}: a C2 coprocessor, or a set
 * of PhotonVision cameras.
 *
 * <p>Each cycle it reads the IO (logging the raw inputs for replay), decodes the results according
 * to the logged {@link NativePacketType}, and hands them out grouped by camera. It also owns the
 * disconnect alerts: one for the source (only if the source reports its own health, like C2) and
 * one per camera.
 *
 * <p>Because the packet type and health capability come from the logged inputs, the same source
 * replays a log from any kind of IO: in replay, just give it a do-nothing IO.
 */
public class VisionSource {
    /**
     * Intrinsic &amp; observed properties describing a camera.
     *
     * @param name Unique name for the camera
     * @param robotToCamera Transform from the robot frame to the camera frame
     * @param cameraMatrix Intrinsic camera matrix
     * @param distCoeffs Distortion coefficients for the camera
     * @param resolutionWidth Camera resolution width in pixels
     * @param resolutionHeight Camera resolution height in pixels
     * @param stdDevFactor Standard deviation factor used in vision pose estimation
     * @param fov Estimated FOV of camera
     * @param fps Estimate FPS of camera
     * @param latency Average latency of the camera (exposure to network tables)
     * @param latencyStdDev Standard deviation of the camera latency
     */
    public record CameraProperties(
            String name,
            Transform3d robotToCamera,
            Matrix<N3, N3> cameraMatrix,
            Matrix<N8, N1> distCoeffs,
            int resolutionWidth,
            int resolutionHeight,
            double stdDevFactor,
            Angle fov,
            double fps,
            Time latency,
            Time latencyStdDev) {}

    /** A camera of this source and the results it produced since the last cycle. */
    public static record CameraResults(CameraProperties camera, List<CameraResult> results) {}

    private static final double DISCONNECT_DEBOUNCE_SECONDS = 0.25;

    private final String name;
    private final List<CameraProperties> cameras;
    private final VisionIO io;
    private final C2Decoder c2Decoder;
    private final PhotonDecoder photonDecoder;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    private final Alert sourceAlert;
    private final Alert unknownFormatAlert;
    private final Debouncer sourceDebouncer = new Debouncer(DISCONNECT_DEBOUNCE_SECONDS);
    private final List<Alert> cameraAlerts = new ArrayList<>();
    private final List<Debouncer> cameraDebouncers = new ArrayList<>();

    /**
     * @param name name of the source, used for logging and alerts. Keep it the same whichever IO is
     *     used, so a log replays from the same key.
     * @param cameras the source's cameras; results and status are indexed in this order
     * @param io where the raw results come from (a do-nothing stub in replay)
     * @param tagLayout field layout used to interpret tag observations
     */
    public VisionSource(
            String name,
            List<CameraProperties> cameras,
            VisionIO io,
            AprilTagFieldLayout tagLayout) {
        this.name = name;
        this.cameras = List.copyOf(cameras);
        this.io = io;
        this.c2Decoder = new C2Decoder(tagLayout);
        this.photonDecoder = new PhotonDecoder(tagLayout);
        this.unknownFormatAlert =
                new Alert(
                        "Vision source " + name + ": unknown packet type, results ignored",
                        AlertType.kError);
        this.sourceAlert =
                new Alert("Vision source " + name + " is not responding!", AlertType.kError);
        for (CameraProperties camera : this.cameras) {
            cameraAlerts.add(
                    new Alert("Camera " + camera.name() + " is Disconnected!", AlertType.kError));
            cameraDebouncers.add(new Debouncer(DISCONNECT_DEBOUNCE_SECONDS));
        }
    }

    /** The source's cameras, in order. */
    public List<CameraProperties> getCameras() {
        return cameras;
    }

    /**
     * Reads and decodes everything new.
     *
     * @return one entry per connected camera, with its results (oldest first)
     */
    public List<CameraResults> getUnreadResults() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision/Sources/" + name, inputs);

        sourceAlert.set(inputs.reportsSourceHealth && sourceDebouncer.calculate(!inputs.connected));

        unknownFormatAlert.set(
                inputs.rawPacketType == NativePacketType.UNKNOWN && inputs.rawResults.length > 0);

        List<List<CameraResult>> byCamera = new ArrayList<>(cameras.size());
        for (int i = 0; i < cameras.size(); i++) {
            byCamera.add(new ArrayList<>());
        }
        for (int j = 0; j < inputs.rawResults.length && j < inputs.resultCameras.length; j++) {
            int camera = inputs.resultCameras[j];
            if (camera < 0 || camera >= cameras.size()) continue;
            long captureUs = valueOrZero(inputs.captureTimestampsUs, j);
            long publishUs = valueOrZero(inputs.publishTimestampsUs, j);
            byte[] raw = inputs.rawResults[j];
            CameraResult result =
                    switch (inputs.rawPacketType) {
                        case C2 -> c2Decoder.decode(raw, captureUs, publishUs);
                        case PHOTON -> photonDecoder.decode(raw, captureUs, publishUs);
                        case UNKNOWN -> null;
                    };
            if (result != null) {
                byCamera.get(camera).add(result);
            }
        }

        List<CameraResults> results = new ArrayList<>(cameras.size());
        for (int i = 0; i < cameras.size(); i++) {
            boolean connected =
                    inputs.connected
                            && i < inputs.camerasConnected.length
                            && inputs.camerasConnected[i];
            updateCameraAlert(i, connected);
            if (connected) {
                results.add(new CameraResults(cameras.get(i), byCamera.get(i)));
            }
        }
        return results;
    }

    private static long valueOrZero(long[] values, int index) {
        return index < values.length ? values[index] : 0;
    }

    private void updateCameraAlert(int camera, boolean connected) {
        String text = "Camera " + cameras.get(camera).name() + " is Disconnected!";
        if (inputs.reportsSourceHealth) {
            // Say why: one camera failing needs a different fix than the coprocessor being down.
            text +=
                    inputs.connected
                            ? " (no images from this camera)"
                            : " (vision source " + name + " not responding)";
        }
        Alert alert = cameraAlerts.get(camera);
        alert.setText(text);
        alert.set(cameraDebouncers.get(camera).calculate(!connected));
    }
}
