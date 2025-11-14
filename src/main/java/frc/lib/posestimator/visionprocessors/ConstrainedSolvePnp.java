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
import org.photonvision.estimation.TargetModel;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.devices.AprilTagCamera.CameraProperties;
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

    private final VisionProcessor seedProvider;
    private final AprilTagFieldLayout fieldLayout;

    @Getter
    @Setter
    private double ambiguityThreshold = DEFAULT_AMBIGUITY_THRESHOLD;

    @Getter
    @Setter
    private double gyroHeadingScaleFactor = DEFAULT_GYRO_HEADING_SCALE_FACTOR;

    @Getter
    @Setter
    private double linearStdDevFactor = DEFAULT_LINEAR_STDDEV_FACTOR;

    @Getter
    @Setter
    private double angularStdDevFactor = DEFAULT_ANGULAR_STDDEV_FACTOR;

    public ConstrainedSolvePnp(VisionProcessor seedProvider, AprilTagFieldLayout fieldLayout)
    {
        this.seedProvider = seedProvider;
        this.fieldLayout = fieldLayout;
    }


    @Override
    public Optional<PoseRecord> processVisionObservation(
        PhotonPipelineResult result,
        CameraProperties camera,
        Rotation2d heading)
    {
        var targets = result.getTargets();
        int tagCount = targets.size();

        // Ignore invalid observations
        if (tagCount == 0) {
            return Optional.empty();
        }

        // Solve for robot pose using constrained PnP
        var robotToCamera = camera.robotToCamera();

        // Attempt to extract seed pose from the observation
        var optionalSeed = seedProvider.processVisionObservation(result, camera, heading);
        if (optionalSeed.isEmpty()) {
            return Optional.empty();
        }
        Pose3d seed = optionalSeed.get().pose();

        Optional<Pose3d> optionalEstimate =
            VisionEstimation.estimateRobotPoseConstrainedSolvepnp(
                camera.cameraMatrix(),
                camera.distCoeffs(),
                targets,
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
        double avgDistance = targets.stream()
            .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
            .average().orElse(0.0);

        double stdDevFactor = (Math.pow(avgDistance, 2.0) / tagCount) * camera.stdDevFactor();
        double linearStdDev = linearStdDevFactor * stdDevFactor;
        double angularStdDev = angularStdDevFactor * stdDevFactor;

        return Optional.of(new PoseRecord(estimate, linearStdDev, angularStdDev));
    }
}
