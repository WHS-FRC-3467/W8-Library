/*
 * Copyright (C) 2025 Windham Windup
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

package frc.lib.posestimator.visionprocessors;

import static edu.wpi.first.units.Units.Meters;
import java.util.Optional;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.devices.apriltagcameras.AprilTagCamera.TagObservation.VisionObservation;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import lombok.experimental.Accessors;

/**
 * A {@link VisionProcessor} implementation that estimates the robot's pose using multi-tag camera
 * pose data produced directly by a coprocessor.
 *
 * <p>
 * This processor assumes that the vision coprocessor provides a precomputed camera pose based on
 * multiple AprilTag detections, and converts that into a field-relative robot pose using the known
 * camera-to-robot transform.
 */
@RequiredArgsConstructor
@Accessors(fluent = true)
public class MultiTagOnCoproc implements VisionProcessor {

    private static final double DEFAULT_LINEAR_STDDEV_BASELINE = 0.025;

    private static final double DEFAULT_ANGULAR_STDDEV_BASELINE = 0.04;

    private final Optional<VisionProcessor> fallbackProcessor;

    private final AprilTagFieldLayout fieldLayout;

    @Getter
    @Setter
    private double linearStdDevBaseline = DEFAULT_LINEAR_STDDEV_BASELINE;

    @Getter
    @Setter
    private double angularStdDevBaseline = DEFAULT_ANGULAR_STDDEV_BASELINE;

    @Override
    public Optional<PoseRecord> processVisionObservation(
        VisionObservation observation,
        Rotation2d heading)
    {
        if (observation.multiTagPose().isEmpty()) {
            if (fallbackProcessor.isEmpty()) {
                return Optional.empty();
            }

            return fallbackProcessor.get().processVisionObservation(observation, heading);
        }

        var best = observation.multiTagPose().get().relativeTo(fieldLayout.getOrigin()); // field-to-robot

        var tagObservations = observation.tagObservations();

        // Compute distance-based uncertainty scaling
        double avgDistance = tagObservations.stream()
            .mapToDouble(target -> target.distance().in(Meters))
            .average().orElse(0.0);

        double stdDevFactor =
            (Math.pow(avgDistance, 2.0) / tagObservations.size())
                * observation.camera().stdDevFactor();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;

        return Optional.of(
            new PoseRecord(
                best,
                linearStdDev,
                angularStdDev));
    }
}
