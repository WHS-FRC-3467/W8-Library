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

import frc.lib.io.vision.VisionIO;
import frc.lib.io.vision.VisionIO.CameraResult;
import frc.lib.io.vision.VisionIOInputsAutoLogged;

import lombok.Getter;

import org.littletonrobotics.junction.Logger;

import java.util.Optional;

/**
 * Represents a single AprilTag camera on the robot.
 *
 * <p>This class owns the IO interface for one physical camera. It calls {@link
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

    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    private final Alert disconnectAlert;
    private final Debouncer disconnectDebouncer = new Debouncer(0.25);

    /** The camera's properties, including intrinsics and transform relative to the robot. */
    @Getter private final CameraProperties properties;

    /**
     * Constructs a new {@code AprilTagCamera}.
     *
     * @param properties the camera's intrinsic and mounting properties
     * @param io the {@link VisionIO} implementation for this camera (handles both raw I/O and
     *     decoding)
     */
    public AprilTagCamera(CameraProperties properties, VisionIO io) {
        this.properties = properties;
        this.io = io;
        this.disconnectAlert =
                new Alert("Camera " + properties.name() + " is Disconnected!", AlertType.kError);
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
        Logger.processInputs(properties.name(), inputs);

        boolean disconnected = !inputs.connected;
        disconnectAlert.set(disconnectDebouncer.calculate(disconnected));
        if (disconnected) return Optional.empty();

        return Optional.of(io.decodeResults(inputs));
    }
}
