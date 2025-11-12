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

import java.util.Optional;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.io.vision.VisionIO.TagObservation;
import frc.lib.io.vision.VisionIO.VisionObservation;
import frc.lib.posestimator.PoseEstimator.VisionProcessor;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import lombok.experimental.Accessors;

/**
 * A {@link VisionProcessor} implementation that estimates the robot's pose using the lowest
 * ambiguity tag observation
 */
@RequiredArgsConstructor
@Accessors(fluent = true)
public class LowestAmbiguity implements VisionProcessor {

    /** Default scaling factor for linear standard deviation (distance-based uncertainty). */
    private static final double DEFAULT_LINEAR_STDDEV_FACTOR = 0.4;

    /** Default scaling factor for angular standard deviation (orientation-based uncertainty). */
    private static final double DEFAULT_ANGULAR_STDDEV_FACTOR = 0.4;

    /** The field layout containing AprilTag locations. */
    private final AprilTagFieldLayout fieldLayout;

    /** Scaling factor for linear standard deviation (distance-based uncertainty). */
    @Getter
    @Setter
    private double linearStdDevFactor = DEFAULT_LINEAR_STDDEV_FACTOR;

    /** Scaling factor for angular standard deviation (orientation-based uncertainty). */
    @Getter
    @Setter
    private double angularStdDevFactor = DEFAULT_ANGULAR_STDDEV_FACTOR;

    /**
     * Processes a {@link VisionObservation} containing multi-tag pose data from the coprocessor and
     * produces an estimated robot pose.
     *
     * <p>
     * If the observation does not include any tag detections, this method returns
     * {@link Optional#empty()}.
     *
     * @param observation The current {@link VisionObservation} containing detected tags and an
     *        optional multi-tag camera pose.
     * @param heading The robot’s current field-relative heading. (Unused in this implementation.)
     * @return An {@link Optional} containing a {@link PoseRecord} with the estimated robot pose, or
     *         empty if no valid observation data was available.
     */
    @Override
    public Optional<PoseRecord> processVisionObservation(VisionObservation observation,
        Rotation2d heading) {

        var tags = observation.tagObservations();
        int tagCount = tags.size();


        // Ignore invalid observations
        if (observation.tagObservations().isEmpty()) {
            return Optional.empty();
        }

        TagObservation lowestAmbiguity = observation.tagObservations().get(0);

        var optionalTargetPose = fieldLayout.getTagPose(lowestAmbiguity.id());
        if (optionalTargetPose.isEmpty()) {
            return Optional.empty();
        }
        Pose3d targetPose = optionalTargetPose.get();

        Transform3d targetToCamera = lowestAmbiguity.cameraToTarget().inverse();
        Transform3d cameraToRobot = observation.camera().robotToCamera().inverse();
        Pose3d robotPose = targetPose.plus(targetToCamera).plus(cameraToRobot);

        double distanceFromCamera = targetToCamera.getTranslation().getNorm();
        double stdDevFactor =
            (Math.pow(distanceFromCamera, 2.0) / tagCount) * observation.camera().stdDevFactor();
        double linearStdDev = linearStdDevFactor * stdDevFactor;
        double angularStdDev = angularStdDevFactor * stdDevFactor;

        return Optional.of(
            new PoseRecord(
                robotPose,
                linearStdDev,
                angularStdDev));
    }
}
