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
import org.photonvision.estimation.TargetModel;
import org.photonvision.estimation.VisionEstimation;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import frc.lib.io.vision.VisionIO.CameraProperties;
import frc.lib.io.vision.VisionIO.TagObservation;
import frc.lib.io.vision.VisionIO.VisionObservation;
import frc.lib.posestimator.PoseEstimator.VisionProcessor;
import frc.lib.posestimator.PoseEstimator.VisionProcessor.PoseRecord;
import frc.lib.util.GeomUtil;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

/**
 * ProcessesAprilTag observations to estimate the robot’s field-relative pose.
 *
 * <p>
 * Each incoming {@link VisionObservation} is evaluated for validity and consistency based on tag
 * count, ambiguity, distance, and field boundaries. The resulting pose estimates are stored or
 * returned as a {@link PoseRecord}, each containing pose data and uncertainty metrics for later
 * fusion with odometry.
 */
@Accessors(fluent = true)
public class ConstrainedSolvePnp implements VisionProcessor {

    private static final double DEFAULT_AMBIGUITY_THRESHOLD = 0.3;
    private static final double DEFAULT_LINEAR_STDDEV_FACTOR = 0.4;
    private static final double DEFAULT_ANGULAR_STDDEV_FACTOR = 0.4;
    private static final double DEFAULT_GYRO_HEADING_SCALE_FACTOR = 10.0;

    private final AprilTagFieldLayout fieldLayout;

    /** Maximum acceptable ambiguity for a single-tag pose estimate. */
    @Getter
    @Setter
    private double ambiguityThreshold = DEFAULT_AMBIGUITY_THRESHOLD;

    /** Weighting factor applied to gyro heading in the constrained PnP solver. */
    @Getter
    @Setter
    private double gyroHeadingScaleFactor = DEFAULT_GYRO_HEADING_SCALE_FACTOR;

    /** Scaling factor for linear standard deviation (distance-based uncertainty). */
    @Getter
    @Setter
    private double linearStdDevFactor = DEFAULT_LINEAR_STDDEV_FACTOR;

    /** Scaling factor for angular standard deviation (orientation-based uncertainty). */
    @Getter
    @Setter
    private double angularStdDevFactor = DEFAULT_ANGULAR_STDDEV_FACTOR;


    /**
     * Constructs a new {@link ConstrainedSolvePnp} using the provided {@link AprilTagFieldLayout}
     *
     * @param fieldLayout The field layout to base calculations on
     */
    public ConstrainedSolvePnp(AprilTagFieldLayout fieldLayout) {
        this.fieldLayout = fieldLayout;
    }

    /**
     * Generates an initial seed pose estimate for the constrained PnP solver using the available
     * vision data.
     *
     * <p>
     * Providing a good seed improves the convergence and accuracy of the constrained PnP solver,
     * especially in multi-tag scenarios or when tags are at oblique viewing angles.
     *
     * @param observation The {@link VisionObservation} containing detected tags and camera data.
     * @return An {@link Optional} containing the estimated robot pose seed in field coordinates, or
     *         empty if insufficient data is available or the tag pose is unknown.
     */
    private Optional<Pose3d> getConstrainedSolvePnPSeedFromVisionObservation(
        VisionObservation observation) {
        Transform3d cameraToRobot = observation.camera().robotToCamera().inverse();

        var optionalMultiTagCameraPose = observation.multiTagCameraPose();
        if (optionalMultiTagCameraPose.isPresent()) {
            Pose3d robotPose = optionalMultiTagCameraPose.get().plus(cameraToRobot);
            return Optional.of(robotPose);
        }

        if (observation.tagObservations().isEmpty()) {
            return Optional.empty();
        }
        TagObservation bestTagObservation = observation.tagObservations().get(0);

        if (bestTagObservation.ambiguity() > ambiguityThreshold) {
            return Optional.empty();
        }

        int tagID = bestTagObservation.id();
        var optionalTagPose = fieldLayout.getTagPose(tagID);
        if (optionalTagPose.isEmpty()) {
            return Optional.empty();
        }
        Pose3d tagPose = optionalTagPose.get();

        Transform3d targetToCamera = bestTagObservation.cameraToTarget().inverse();
        Pose3d robotPose = tagPose.plus(targetToCamera).plus(cameraToRobot);

        return Optional.of(robotPose);
    }

    /**
     * Processes a complete set of vision tag detections and computes a 3D pose estimate if valid.
     *
     * @param observation The {@link VisionObservation} containing all AprilTag detections.
     * @param heading The robot’s field-relative heading at the observation time.
     * @return An {@link Optional} containing a {@link PoseRecord} with the estimated pose and
     *         associated uncertainty, or empty if the observation is invalid or unreliable.
     */
    @Override
    public Optional<PoseRecord> processVisionObservation(VisionObservation observation,
        Rotation2d heading) {
        var tags = observation.tagObservations();
        CameraProperties camera = observation.camera();
        int tagCount = tags.size();

        // Ignore invalid observations
        if (tagCount == 0) {
            return Optional.empty();
        }

        // Solve for robot pose using constrained PnP
        var photonTargets = tags.stream().map(TagObservation::toPhotonTarget).toList();
        var robotToCamera = camera.robotToCamera();

        // Attempt to extract seed pose from the observation
        var optionalSeed = getConstrainedSolvePnPSeedFromVisionObservation(observation);
        if (optionalSeed.isEmpty()) {
            return Optional.empty();
        }
        Pose3d seed = optionalSeed.get();

        Optional<Pose3d> optionalEstimate =
            VisionEstimation.estimateRobotPoseConstrainedSolvepnp(
                camera.cameraMatrix(),
                camera.distCoeffs(),
                photonTargets,
                robotToCamera,
                seed,
                fieldLayout,
                TargetModel.kAprilTag36h11,
                false,
                heading,
                gyroHeadingScaleFactor)
                .map(estimate -> GeomUtil.toPose3d(estimate.best));

        if (optionalEstimate.isEmpty()) {
            return Optional.empty();
        }

        Pose3d estimate = optionalEstimate.get();

        // Compute distance-based uncertainty scaling
        double avgDistance = tags.stream()
            .mapToDouble(tag -> tag.distance().in(Meters))
            .average().orElse(0.0);

        double stdDevFactor = (Math.pow(avgDistance, 2.0) / tagCount) * camera.stdDevFactor();
        double linearStdDev = linearStdDevFactor * stdDevFactor;
        double angularStdDev = angularStdDevFactor * stdDevFactor;

        return Optional.of(new PoseRecord(estimate, linearStdDev, angularStdDev));
    }
}
