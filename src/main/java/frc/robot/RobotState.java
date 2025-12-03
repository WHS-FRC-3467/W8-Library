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

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import java.util.HashMap;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.posestimator.PoseEstimator;
import frc.lib.posestimator.PoseEstimator.VisionPoseObservation;
import frc.lib.posestimator.SwerveOdometry.OdometryObservation;
import frc.robot.subsystems.drive.Drive;
import lombok.AccessLevel;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class RobotState {
    public static record TrigPoseRecord(Pose2d pose, double distance, double timestamp) {
    }

    private static final double LINEAR_ODOMETRY_STD_DEV = 0.01;
    private static final double ANGULAR_ODOMETRY_STD_DEV = 0.01;
    private static final double TRIG_POSE_STALE_SECS = 0.2;

    @Getter(lazy = true)
    private static final RobotState instance = new RobotState();

    private HashMap<Integer, TrigPoseRecord> trigPoses = new HashMap<>();

    private final PoseEstimator poseEstimator = new PoseEstimator(
        new SwerveDriveKinematics(Drive.getModuleTranslations()),
        Seconds.of(2),
        LINEAR_ODOMETRY_STD_DEV,
        ANGULAR_ODOMETRY_STD_DEV);

    @Getter
    @Setter
    private ChassisSpeeds velocity = new ChassisSpeeds();

    @AutoLogOutput(key = "Odometry/OdometryPose")
    public Pose2d getOdometryPose()
    {
        return poseEstimator.odometryPose();
    }

    @AutoLogOutput(key = "Odometry/EstimatedPose")
    public Pose2d getEstimatedPose()
    {
        return poseEstimator.estimatedPose();
    }

    @AutoLogOutput(key = "Odometry/TrigTestPose")
    public Pose2d getTrigTestPose()
    {
        return getTrigPose(10).orElse(Pose2d.kZero);
    }

    public void addOdometryObservation(OdometryObservation observation)
    {
        poseEstimator.addOdometryObservation(observation);
    }

    public void addVisionObservation(VisionPoseObservation observation)
    {
        poseEstimator.addVisionObservation(observation);
    }

    public void addTrigPose(int tagId, TrigPoseRecord trigPose)
    {
        trigPoses.put(tagId, trigPose);
    }

    public Optional<Pose2d> getPoseAtTime(double timestampSeconds)
    {
        return poseEstimator.getPoseAtTime(timestampSeconds);
    }

    public Optional<Pose2d> getTrigPose(int tagId)
    {
        if (!trigPoses.containsKey(tagId)) {
            return Optional.empty();
        }
        var data = trigPoses.get(tagId);

        if (Timer.getTimestamp() - data.timestamp() >= TRIG_POSE_STALE_SECS) {
            return Optional.empty();
        }

        return poseEstimator.getPoseDeltaThenToNow(data.timestamp())
            .map(delta -> data.pose().transformBy(delta));
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation()
    {
        return getEstimatedPose().getRotation();
    }

    public ChassisSpeeds getFieldRelativeVelocity()
    {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            velocity.vxMetersPerSecond,
            velocity.vyMetersPerSecond,
            velocity.omegaRadiansPerSecond,
            getRotation());
    }

    @Getter
    @Setter
    private Pose3d rotaryPose = new Pose3d();

    @Getter
    @Setter
    private Pose3d linearPose = new Pose3d();

    /**
     * Publishes the mechanism poses to the logger for 3d visualization. This should be changed to
     * match the mechanical kinematics of the robot.
     */

    public void publishMechanismPoses()
    {
        Logger.recordOutput("Odometry/LinearPose", linearPose);
        Logger.recordOutput("Odometry/RotaryPose", new Pose3d(
            getRotaryPose().getX(),
            getRotaryPose().getY(),
            getRotaryPose().getZ() + getLinearPose().getZ(),
            getRotaryPose().getRotation()));
    }

    public void resetPose(Pose2d pose)
    {
        poseEstimator.resetPose(pose);
    }
}
