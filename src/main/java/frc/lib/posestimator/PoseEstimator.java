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

package frc.lib.posestimator;

import frc.lib.posestimator.SwerveOdometry.OdometryObservation;
import java.util.List;
import java.util.Optional;
import java.util.function.Predicate;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Fuses swerve odometry with vision measurements into a field-relative pose estimate.
 *
 * <p>Implementations: {@link MechAPoseEstimator} (WPILib's fixed per-axis gain method) and {@link
 * EkfPoseEstimator} (extended Kalman filter with covariance tracking and outlier gating).
 */
public interface PoseEstimator {

    /**
     * A robot pose measured by vision.
     *
     * @param timestampSeconds when the frame was captured, in the odometry timebase
     * @param robotPose the field-relative robot pose measured by vision
     * @param avgTagDistance average distance to the tags used, in meters
     * @param seenTagIds the ids of the tags used
     * @param linearStdDev standard deviation of the x and y measurement, in meters
     * @param angularStdDev standard deviation of the heading measurement, in radians (infinite to
     *     ignore heading)
     */
    record VisionPoseObservation(
            double timestampSeconds,
            Pose2d robotPose,
            double avgTagDistance,
            List<Integer> seenTagIds,
            double linearStdDev,
            double angularStdDev) {}

    /**
     * Returns the current best estimate of the robot's pose, with vision corrections applied.
     *
     * @return the estimated pose
     */
    Pose2d estimatedPose();

    /**
     * Returns the odometry-only pose, with no vision corrections.
     *
     * @return the odometry pose
     */
    Pose2d odometryPose();

    /**
     * Adds an odometry observation (module positions and optional gyro angle) and updates the
     * estimate.
     *
     * @param observation the odometry observation
     */
    void addOdometryObservation(OdometryObservation observation);

    /**
     * Adds an observation that updates only heading from the gyro, ignoring module positions.
     *
     * @param observation an observation containing an optional gyro angle
     */
    void addGyroObservation(OdometryObservation observation);

    /**
     * Adds a vision measurement, applying it at its capture time and carrying the correction
     * forward to the present.
     *
     * @param observation the vision observation
     */
    void addVisionObservation(VisionPoseObservation observation);

    /**
     * Returns the estimated pose at a past time.
     *
     * @param timestampSeconds the time to look up, in the odometry timebase
     * @return the pose then, or empty if the history does not reach that time
     */
    Optional<Pose2d> getPoseAtTime(double timestampSeconds);

    /**
     * Returns the robot-relative velocity around a past time, from odometry.
     *
     * @param timestampSeconds the time to look up, in the odometry timebase
     * @return the velocity then, or empty if the history does not reach that time
     */
    Optional<ChassisVelocities> getVelocityAtTime(double timestampSeconds);

    /**
     * Resets the estimate and odometry to a known pose.
     *
     * @param pose the new pose
     */
    void resetPose(Pose2d pose);

    /**
     * Sets a validator that rejects implausible poses (for example, outside the field).
     *
     * @param validator predicate that returns {@code true} for acceptable poses
     * @return this estimator
     */
    PoseEstimator withPoseValidator(Predicate<Pose2d> validator);
}
