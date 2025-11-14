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
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.devices.AprilTagCamera.CameraProperties;
import frc.lib.posestimator.PoseEstimator;
import frc.lib.posestimator.SwerveOdometry.OdometryObservation;
import frc.lib.posestimator.visionprocessors.ConstrainedSolvePnp;
import frc.lib.posestimator.visionprocessors.LowestAmbiguity;
import frc.lib.posestimator.visionprocessors.MultiTagOnCoproc;
import frc.lib.posestimator.visionprocessors.VisionProcessor.PoseRecord;
import frc.robot.subsystems.drive.Drive;
import lombok.AccessLevel;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class RobotState {
    private static final double MAX_Z_METERS = 0.75;
    private static final double FIELD_LENGTH = FieldConstants.aprilTagLayout.getFieldLength();
    private static final double FIELD_WIDTH = FieldConstants.aprilTagLayout.getFieldLength();

    private static final double LINEAR_ODOMETRY_STD_DEV = 0.01;
    private static final double ANGULAR_ODOMETRY_STD_DEV = 0.01;

    @Getter(lazy = true)
    private static final RobotState instance = new RobotState();

    private static boolean postFilter(PoseRecord poseRecord)
    {
        Pose3d pose = poseRecord.pose();
        double x = pose.getX();
        double y = pose.getY();
        double z = pose.getZ();
        return z > MAX_Z_METERS || x < 0.0 || x > FIELD_LENGTH || y < 0.0 || y > FIELD_WIDTH;
    }

    private final LowestAmbiguity fallbackVisionProcessor =
        new LowestAmbiguity(FieldConstants.aprilTagLayout);
    private final MultiTagOnCoproc seedVisionProcessor =
        new MultiTagOnCoproc(
            Optional.of(fallbackVisionProcessor),
            FieldConstants.aprilTagLayout);
    private final ConstrainedSolvePnp visionProcessor =
        new ConstrainedSolvePnp(seedVisionProcessor, FieldConstants.aprilTagLayout);

    private final PoseEstimator poseEstimator = new PoseEstimator(
        visionProcessor,
        new SwerveDriveKinematics(Drive.getModuleTranslations()),
        Seconds.of(2),
        LINEAR_ODOMETRY_STD_DEV,
        ANGULAR_ODOMETRY_STD_DEV)
            .visionPoseFilter(Optional.of(RobotState::postFilter));

    @Getter
    @AutoLogOutput(key = "Odometry/Robot")
    private Pose2d estimatedPose = Pose2d.kZero;

    @Getter
    private Optional<PhotonTrackedTarget> closestTagObservation = Optional.empty();

    @Getter
    @Setter
    private ChassisSpeeds velocity = new ChassisSpeeds();

    public void addOdometryObservation(OdometryObservation observation)
    {
        poseEstimator.addOdometryObservation(observation);
        estimatedPose = poseEstimator.estimatedPose();
    }

    public void addVisionObservation(PhotonPipelineResult observation, CameraProperties camera)
    {
        closestTagObservation = observation.getTargets().stream().sorted((t1, t2) -> {
            double t1Distance = t1.getBestCameraToTarget().getTranslation().getNorm();
            double t2Distance = t2.getBestCameraToTarget().getTranslation().getNorm();
            if (t2Distance < t1Distance)
                return -1;
            if (t2Distance > t1Distance)
                return 1;
            return 0;
        }).findFirst();
        poseEstimator.addVisionObservation(observation, camera);
        estimatedPose = poseEstimator.estimatedPose();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation()
    {
        return estimatedPose.getRotation();
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
        estimatedPose = poseEstimator.estimatedPose();
    }
}
