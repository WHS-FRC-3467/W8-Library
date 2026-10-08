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
import edu.wpi.first.wpilibj.DriverStation;

import frc.lib.io.vision.VisionIO;
import frc.lib.io.vision.VisionIO.CameraResult;
import frc.lib.io.vision.VisionIOC2;
import frc.lib.io.vision.VisionIOInputsAutoLogged;
import frc.lib.io.vision.VisionIOPhotonVision;

import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.Optional;

/**
 * Owns one vision IO source and the properties for the robot's physical cameras.
 *
 * <p>This class calls {@link
 * VisionIO#updateInputs} each cycle to flush raw bytes into the AdvantageKit-logged {@link
 * frc.lib.io.vision.VisionIO.VisionIOInputs}, then delegates decoding to {@link
 * VisionIO#decodeResults} so that all format-specific logic stays in the IO layer.
 */
public class AprilTagCamera {
    /**
     * Intrinsic &amp; observed properties describing the camera.
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
            int index,
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

    private final String name;
    private final CameraProperties[] cameraProperties;
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private final Alert disconnectAlert;
    private final Debouncer disconnectDebouncer = new Debouncer(0.25);

    private final AprilTagFieldLayout fieldLayout;

    /** Constructs one robot-level IO owner with properties for every physical camera. */
    public AprilTagCamera(
            String name,
            CameraProperties[] cameraProperties,
            VisionIO io,
            AprilTagFieldLayout fieldLayout) {
        if (cameraProperties == null || cameraProperties.length == 0) {
            throw new IllegalArgumentException("At least one camera property is required");
        }
        this.name = name;
        this.cameraProperties = Arrays.copyOf(cameraProperties, cameraProperties.length);
        this.io = io;
        this.fieldLayout = fieldLayout;
        this.disconnectAlert =
                new Alert("Vision source " + name + " is disconnected!", AlertType.kError);
    }

    public CameraProperties getProperties(int cameraIndex) {
        return Arrays.stream(cameraProperties)
                .filter(properties -> properties.index() == cameraIndex)
                .findFirst()
                .orElseThrow(
                        () ->
                                new IllegalArgumentException(
                                        "Unknown camera index " + cameraIndex + " on robot"));
    }

    public CameraProperties[] getAllCameraProperties() {
        return Arrays.copyOf(cameraProperties, cameraProperties.length);
    }

    /**
     * Polls the camera for new results.
     *
     * <p>Calls {@link VisionIO#updateInputs} to read raw bytes (logged by AdvantageKit for replay),
     * then calls {@link VisionIO#decodeResults} to convert them to {@link CameraResult} records.
     *
     * @return an {@link Optional} containing decoded results, or {@link Optional#empty()} if the
     *     camera is disconnected
     */
    public Optional<CameraResult[]> getUnreadResults() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);

        boolean disconnected = !inputs.connected;
        disconnectAlert.set(disconnectDebouncer.calculate(disconnected));
        if (disconnected) return Optional.empty();

        return switch (inputs.rawPacketType) {
            case UNKNOWN -> {
                if (inputs.rawResults.length == 0) {
                    yield Optional.of(new CameraResult[0]);
                }
                DriverStation.reportError(
                        "Unknown/Invalid Vision Packet Type Reported. Ignoring.", null);
                yield Optional.empty();
            }
            case C2 -> Optional.of(VisionIOC2.decodeResults(inputs, fieldLayout));
            case PHOTON ->
                    Optional.of(
                            VisionIOPhotonVision.decodeResults(
                                    inputs,
                                    fieldLayout,
                                    cameraProperties.length == 0
                                            ? -1
                                            : cameraProperties[0].index()));
        };
    }
}
