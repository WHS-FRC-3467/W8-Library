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
import org.littletonrobotics.junction.AutoLogOutput;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.io.vision.VisionIO.VisionObservation;
import frc.lib.posestimator.PoseEstimator;
import frc.lib.posestimator.PoseEstimator.OdometryObservation;
import frc.lib.util.Timestamped;
import frc.robot.subsystems.drive.Drive;
import lombok.AccessLevel;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class RobotState {
    @Getter(lazy = true)
    private static final RobotState instance = new RobotState();

    private final PoseEstimator poseEstimator = new PoseEstimator(
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark),
        new SwerveDriveKinematics(Drive.getModuleTranslations()),
        Seconds.of(2));

    @Getter
    @AutoLogOutput(key = "Odometry/Robot")
    private Pose2d estimatedPose = Pose2d.kZero;
    @AutoLogOutput(key = "Odometry/Test")
    private Pose2d testPose = Pose2d.kZero;

    @Getter
    @Setter
    private ChassisSpeeds velocity = new ChassisSpeeds();

    public void addOdometryObservation(OdometryObservation observation)
    {
        poseEstimator.addOdometryObservation(observation);
        estimatedPose = poseEstimator.getEstimatedPose();
        testPose = poseEstimator.getTrigPose(10).orElse(Pose2d.kZero);
    }

    public void addVisionObservation(VisionObservation observation)
    {
        poseEstimator.addVisionObservation(observation);
        estimatedPose = poseEstimator.getEstimatedPose();
        testPose = poseEstimator.getTrigPose(10).orElse(Pose2d.kZero);
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation()
    {
        return estimatedPose.getRotation();
    }

    public Timestamped<Rotation2d> getTimestampedHeading()
    {
        return new Timestamped<Rotation2d>(Seconds.of(Timer.getTimestamp()), getRotation());
    }

    public ChassisSpeeds getFieldRelativeVelocity()
    {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            velocity.vxMetersPerSecond,
            velocity.vyMetersPerSecond,
            velocity.omegaRadiansPerSecond,
            getRotation());
    }

    public void resetPose(Pose2d pose)
    {
        this.estimatedPose = pose;
    }
}
