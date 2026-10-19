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

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.LinearVelocity;

import frc.lib.posestimator.EkfPoseEstimator;
import frc.lib.posestimator.PoseEstimator.VisionPoseObservation;
import frc.lib.posestimator.SwerveOdometry.OdometryObservation;
import frc.robot.subsystems.drive.Drive;

import lombok.AccessLevel;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

import org.littletonrobotics.junction.AutoLogOutput;

import java.util.Optional;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class RobotState {

    @Getter(lazy = true)
    private static final RobotState instance = new RobotState();

    @AutoLogOutput(key = "Drive/ActiveTrajectoryPose")
    @Getter
    @Setter
    private Pose2d activeTrajPose = new Pose2d();

    // -------- POSE ESTIMATION --------

    private final EkfPoseEstimator poseEstimator =
            new EkfPoseEstimator(
                    new SwerveDriveKinematics(
                            Drive.MODULE_TRANSLATIONS.toArray(Translation2d[]::new)));

    @Getter @Setter private ChassisSpeeds robotRelativeVelocity = new ChassisSpeeds();

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
        // if (DriverStation.isDisabled()) return;

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
     * Returns the robot's robot-relative velocity around a past time, so a vision frame can be
     * judged by how fast the robot was moving when it was captured.
     *
     * @param timestampSeconds the time to look up, in the odometry timebase
     * @return the velocity then, or the current velocity if the history does not reach that time
     */
    public ChassisSpeeds getRobotRelativeVelocityAtTime(double timestampSeconds) {
        return poseEstimator.getVelocityAtTime(timestampSeconds).orElse(robotRelativeVelocity);
    }

    /**
     * Returns the robot's field-relative velocity.
     *
     * @return the field-relative chassis speeds
     */
    public ChassisSpeeds getFieldRelativeVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelativeVelocity.vxMetersPerSecond,
                robotRelativeVelocity.vyMetersPerSecond,
                robotRelativeVelocity.omegaRadiansPerSecond,
                getEstimatedPose().getRotation());
    }

    /**
     * Returns the robot's linear velocity.
     *
     * @return the linear velocity of the robot
     */
    public LinearVelocity getLinearVelocity() {
        var speeds = getFieldRelativeVelocity();
        return MetersPerSecond.of(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
    }

    /**
     * Resets the robot's pose to the specified position.
     *
     * @param pose the new pose to set
     */
    public void resetPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    /**
     * Returns the robot's estimated position {@code seconds} in the future
     *
     * @param seconds amount of time to predict
     * @return the robot's estimated position {@code seconds} in the future
     */
    // public Pose2d getFuturePose(double seconds) {
    // Transform2d velocity = new Transform2d(
    // robotRelativeVelocity.vxMetersPerSecond,
    // robotRelativeVelocity.vyMetersPerSecond,
    // Rotation2d.fromRadians(robotRelativeVelocity.omegaRadiansPerSecond));
    // return getEstimatedPose().plus(velocity.times(feedLookaheadSeconds.get()));
    // }
}
