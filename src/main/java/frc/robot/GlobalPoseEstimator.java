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

package frc.robot;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;

import frc.lib.posestimator.EkfPoseEstimator;
import frc.lib.posestimator.PoseEstimator.VisionPoseObservation;
import frc.lib.posestimator.SwerveOdometry.OdometryObservation;
import frc.lib.util.FieldUtil;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import lombok.AccessLevel;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.units.measure.LinearVelocity;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class GlobalPoseEstimator {

    @Getter(lazy = true)
    private static final GlobalPoseEstimator instance = new GlobalPoseEstimator();

    @AutoLogOutput(key = "Drive/ActiveTrajectoryPose")
    @Getter
    @Setter
    private Pose2d activeTrajPose = new Pose2d();

    // -------- POSE ESTIMATION --------

    private final EkfPoseEstimator poseEstimator = new EkfPoseEstimator(
                    new SwerveDriveKinematics(Drive.MODULE_TRANSLATIONS.toArray(Translation2d[]::new)))
            .withPoseValidator(p -> FieldUtil.isPoseInField(p.getTranslation(), Meters.zero()));

    @Getter
    @Setter
    private ChassisVelocities robotRelativeVelocity = new ChassisVelocities();

    /**
     * Returns the robot's odometry-only pose (without vision corrections).
     *
     * @return the odometry-only pose
     */
    @AutoLogOutput(key = "Odometry/OdometryPose")
    public Pose2d getOdometryPose() {
        return poseEstimator.odometryPose();
    }

    /**
     * Returns the robot's estimated pose with vision corrections applied.
     *
     * @return the estimated pose
     */
    @AutoLogOutput(key = "Odometry/EstimatedPose")
    public Pose2d getEstimatedPose() {
        return poseEstimator.estimatedPose();
    }

    /**
     * Adds a new odometry observation to the pose estimator.
     *
     * @param observation the odometry observation to add
     */
    public void addOdometryObservation(OdometryObservation observation) {
        // if (RobotState.isDisabled()) return;

        poseEstimator.addOdometryObservation(observation);
    }

    /**
     * Tells the pose estimator whether the robot is tipped. While tipped, odometry is trusted less,
     * so vision pulls the pose back harder, during the tip and in the first frames after it. Call
     * every cycle, before that cycle's odometry observations.
     *
     * @param tipped whether the robot is tipped
     */
    public void setTipped(boolean tipped) {
        poseEstimator.setTipped(tipped);
    }

    /**
     * Adds a new vision observation to the pose estimator.
     *
     * @param observation the vision observation to add
     */
    public void addVisionObservation(VisionPoseObservation observation) {
        poseEstimator.addVisionObservation(observation);
    }

    /**
     * Returns the robot's estimated pose at a specific timestamp.
     *
     * @param timestampSeconds the timestamp in seconds
     * @return the estimated pose at the given timestamp, or empty if unavailable
     */
    public Optional<Pose2d> getPoseAtTime(double timestampSeconds) {
        return poseEstimator.getPoseAtTime(timestampSeconds);
    }

    /**
     * Returns the robot-relative velocity around a past timestamp, from odometry.
     *
     * @param timestampSeconds the timestamp in seconds
     * @return the velocity then, or the current velocity if the history does not reach that time
     */
    public ChassisVelocities getRobotRelativeVelocityAtTime(double timestampSeconds) {
        return poseEstimator.getVelocityAtTime(timestampSeconds).orElse(robotRelativeVelocity);
    }

    /**
     * Returns the robot's field-relative velocity.
     *
     * @return the field-relative chassis speeds
     */
    public ChassisVelocities getFieldRelativeVelocity() {
        return robotRelativeVelocity.toFieldRelative(getEstimatedPose().getRotation());
    }

    /**
     * Returns the robot's linear velocity.
     *
     * @return the linear velocity of the robot
     */
    public LinearVelocity getLinearVelocity() {
        var speeds = getFieldRelativeVelocity();
        return MetersPerSecond.of(Math.hypot(speeds.vx, speeds.vy));
    }

    /**
     * Resets the robot's pose to the specified position.
     *
     * @param pose the new pose to set
     */
    public void resetPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }
}
