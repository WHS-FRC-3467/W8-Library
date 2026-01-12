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

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.devices.AprilTagCamera;
import frc.lib.posestimator.PoseEstimator.VisionPoseObservation;
import frc.lib.posestimator.visionprocessors.LowestAmbiguity;
import frc.lib.posestimator.visionprocessors.MultiTagOnCoproc;
import frc.lib.posestimator.visionprocessors.TrigSolve;
import frc.lib.posestimator.visionprocessors.VisionProcessor.VisionPoseRecord;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.TrigPoseRecord;

/**
 * The {@code VisionSubsystem} manages all vision-related processing for the robot.
 * 
 * <p>
 * It uses one or more {@link AprilTagCamera}s to detect field elements and estimate the robot's
 * pose on the field. Observations are processed through the {@link MultiTagOnCoproc} vision
 * processor, with a fallback to {@link LowestAmbiguity} if necessary. Valid observations are added
 * to {@link RobotState} for use in localization and navigation.
 * 
 * <p>
 * The subsystem periodically polls cameras for new results and logs both accepted and rejected
 * vision observations.
 */
public class VisionSubsystem extends SubsystemBase {

    public static Transform3d test = new Transform3d();

    public static final String LOG_PREFIX = "VisionProcessor/";

    public static final double LINEAR_STDDEV_BASELINE = 0.4;
    public static final double ANGULAR_STDDEV_BASELINE = 0.4;
    public static final double MAX_Z_METERS = 0.75;
    public static final double MAX_DISTANCE_METERS = 10;
    public static final double MAX_AMBIGUITY = 0.2;

    public static final double FIELD_WIDTH = FieldConstants.FIELD_WIDTH.in(Meters);
    public static final double FIELD_LENGTH = FieldConstants.FIELD_LENGTH.in(Meters);

    private final RobotState robotState = RobotState.getInstance();
    private final AprilTagCamera[] cameras;

    private final LowestAmbiguity fallbackVisionProcessor =
        new LowestAmbiguity(FieldConstants.APRILTAG_LAYOUT);
    private final MultiTagOnCoproc visionProcessor =
        new MultiTagOnCoproc(
            Optional.of(fallbackVisionProcessor),
            FieldConstants.APRILTAG_LAYOUT);

    public VisionSubsystem(AprilTagCamera... cameras)
    {
        this.cameras = cameras;
        setDefaultCommand(visionProcessingCommand());
    }

    /**
     * Returns a command that continuously processes vision data from all cameras and adds valid
     * pose observations to {@link RobotState}.
     */
    public Command visionProcessingCommand()
    {
        return run(() -> {

            for (var camera : cameras) {
                String cameraLogPrefix = LOG_PREFIX + camera.getProperties().name() + "/";

                PhotonPipelineResult[] results = camera.getUnreadResults().orElse(null);
                if (results == null) {
                    continue;
                }

                ArrayList<PhotonPipelineResult> acceptedResults = new ArrayList<>();
                ArrayList<PhotonPipelineResult> rejectedResults = new ArrayList<>();
                ArrayList<Pose2d> acceptedPoses = new ArrayList<>();
                ArrayList<Pose2d> rejectedPoses = new ArrayList<>();

                for (var result : results) {
                    // if (!preFilter(result)) {
                    // rejectedResults.add(result);
                    // continue;
                    // }

                    Rotation2d heading = robotState.getPoseAtTime(result.getTimestampSeconds())
                        .map(Pose2d::getRotation).orElse(null);
                    if (heading == null) {
                        rejectedResults.add(result);
                        continue;
                    }

                    result.targets.forEach(target -> {
                        Pose2d pose = TrigSolve.solveTrigPosition(FieldConstants.APRILTAG_LAYOUT,
                            camera.getProperties(), target, heading).orElse(null);
                        if (pose == null /* || !postFilter(new Pose3d(pose)) */) {
                            return;
                        }

                        robotState.addTrigPose(
                            target.getFiducialId(),
                            new TrigPoseRecord(
                                pose,
                                target.getBestCameraToTarget().getTranslation().getNorm(),
                                result.getTimestampSeconds()));
                    });

                    VisionPoseRecord poseRecord = visionProcessor.processVisionObservation(
                        result,
                        camera.getProperties(),
                        heading)
                        .orElse(null);

                    if (poseRecord == null) {
                        rejectedResults.add(result);
                        continue;
                    }

                    // if (!postFilter(poseRecord.pose())) {
                    // rejectedResults.add(result);
                    // rejectedPoses.add(
                    // poseRecord.pose().toPose2d());
                    // continue;
                    // }

                    double stdDevFactor =
                        (Math.pow(poseRecord.averageDistanceMeters(), 2.0)
                            / result.getTargets().size())
                            * camera.getProperties().stdDevFactor();
                    double linearStdDev = LINEAR_STDDEV_BASELINE * stdDevFactor;
                    double angularStdDev = ANGULAR_STDDEV_BASELINE * stdDevFactor;

                    robotState.addVisionObservation(
                        new VisionPoseObservation(
                            result.getTimestampSeconds(),
                            poseRecord.pose().toPose2d(),
                            linearStdDev,
                            angularStdDev));

                    acceptedResults.add(result);
                    acceptedPoses.add(
                        poseRecord.pose().toPose2d());
                }

                Logger.recordOutput(
                    cameraLogPrefix + "/Results/AcceptedLength",
                    acceptedResults.size());
                for (int i = 0; i < acceptedResults.size(); i++) {
                    Logger.recordOutput(
                        cameraLogPrefix + "/Results/Accepted/" + i,
                        acceptedResults.get(i));
                }

                Logger.recordOutput(
                    cameraLogPrefix + "/Results/RejectedLength",
                    rejectedResults.size());
                for (int i = 0; i < rejectedResults.size(); i++) {
                    Logger.recordOutput(
                        cameraLogPrefix + "/Results/Rejected/" + i,
                        rejectedResults.get(i));
                }

                Logger.recordOutput(
                    cameraLogPrefix + "/Poses/AcceptedLength",
                    acceptedPoses.size());
                for (int i = 0; i < acceptedPoses.size(); i++) {
                    Logger.recordOutput(
                        cameraLogPrefix + "/Poses/Accepted/" + i,
                        acceptedPoses.get(i));
                }

                Logger.recordOutput(
                    cameraLogPrefix + "/Poses/RejectedLength",
                    rejectedPoses.size());
                for (int i = 0; i < rejectedPoses.size(); i++) {
                    Logger.recordOutput(
                        cameraLogPrefix + "/Results/Rejected/" + i,
                        rejectedPoses.get(i));
                }
            }
        });
    }

    /**
     * Creates a command that performs camera extrinsic calibration.
     *
     * <p>
     * The returned command repeatedly collects samples of the camera-to-target {@link Transform3d}
     * reported by each AprilTag camera while the robot is held at a known calibration point on the
     * field. For each camera, the command accumulates {@code SAMPLE_COUNT} camera-to-target
     * transform samples, uses the known transform from the target to the robot center to infer the
     * camera's pose relative to the robot, and then averages those inferred poses to produce an
     * extrinsic calibration for each camera. The resulting calibration data is written to
     * NetworkTables so it can be inspected or saved by external tooling.
     *
     * <p>
     * The command finishes automatically after it has collected {@code SAMPLE_COUNT} valid samples
     * from <em>all</em> cameras in {@link #cameras}.
     *
     * @param primaryCameraIndex index into {@code cameras[]} designating the primary camera whose
     *        target observations are used as the reference during calibration
     * @param robotCenterTransform known transform from the calibration target to the robot center
     *        when the robot is positioned at the calibration point
     */
    public Command cameraCalibrationCommand(
        int primaryCameraIndex,
        Transform3d robotCenterTransform)
    {
        return new Command() {
            private static final int SAMPLE_COUNT = 1000;

            private final AprilTagCamera primary = cameras[primaryCameraIndex];

            private final Map<AprilTagCamera, ArrayList<Transform3d>> samples =
                new HashMap<>();

            // Non-static initalizer
            {
                addRequirements(VisionSubsystem.this);
            }

            @Override
            public void initialize()
            {
                for (var cam : cameras) {
                    samples.put(cam, new ArrayList<>());
                }
            }

            @Override
            public void execute()
            {
                for (var camera : cameras) {
                    if (samples.get(camera).size() >= SAMPLE_COUNT) {
                        continue;
                    }

                    PhotonPipelineResult[] results =
                        camera.getUnreadResults().orElse(null);
                    if (results == null) {
                        continue;
                    }

                    for (var result : results) {
                        if (!result.hasTargets()) {
                            continue;
                        }

                        result.getTargets().forEach(target -> {
                            if (samples.get(camera).size() >= SAMPLE_COUNT) {
                                return;
                            }
                            samples.get(camera)
                                .add(target.getBestCameraToTarget());
                        });
                    }
                }
            }

            @Override
            public boolean isFinished()
            {
                return samples.values().stream()
                    .allMatch(list -> list.size() >= SAMPLE_COUNT);
            }

            @Override
            public void end(boolean interrupted)
            {
                if (interrupted) {
                    return;
                }

                // Estimate primary camera -> target
                Transform3d primaryCamToTarget =
                    average(samples.get(primary));

                Transform3d primaryCamToRobot = primaryCamToTarget.plus(robotCenterTransform);

                test = primaryCamToRobot;

                Logger.recordOutput(
                    "VisionCalibration/PrimaryCameraToRobot",
                    primaryCamToRobot);

                // Other cameras: camera -> primary -> robot
                for (var camera : cameras) {
                    if (camera == primary) {
                        continue;
                    }

                    Transform3d camToTarget =
                        average(samples.get(camera));

                    Transform3d camToPrimary =
                        camToTarget.plus(primaryCamToTarget.inverse());

                    Transform3d camToRobot =
                        camToPrimary.plus(primaryCamToRobot);

                    Logger.recordOutput(
                        "VisionCalibration/" + camera.getProperties().name()
                            + "/CameraToRobot",
                        camToRobot);
                }
            }

            /**
             * Computes an average of a list of transforms.
             *
             * <p>
             * Translation components (x, y, z) are averaged using a simple arithmetic mean.
             * Rotation is averaged by summing the quaternions (with hemisphere correction to keep
             * them on a consistent side of the unit sphere) and then normalizing the result to
             * produce the final {@link Rotation3d}.
             */
            private Transform3d average(ArrayList<Transform3d> list)
            {
                double x = 0;
                double y = 0;
                double z = 0;

                double qw = 0;
                double qx = 0;
                double qy = 0;
                double qz = 0;

                for (var t : list) {
                    x += t.getX();
                    y += t.getY();
                    z += t.getZ();

                    // Quaternion averaging (Unfortunately no WPILib built-in)
                    var q = t.getRotation().getQuaternion();

                    // Hemisphere correction
                    if (q.getW() < 0) {
                        qw -= q.getW();
                        qx -= q.getX();
                        qy -= q.getY();
                        qz -= q.getZ();
                    } else {
                        qw += q.getW();
                        qx += q.getX();
                        qy += q.getY();
                        qz += q.getZ();
                    }
                }

                int n = list.size();

                var avgQuat = new Quaternion(
                    qw / n,
                    qx / n,
                    qy / n,
                    qz / n).normalize();

                return new Transform3d(
                    x / n,
                    y / n,
                    z / n,
                    new Rotation3d(avgQuat));
            }
        };
    }

    public static boolean preFilter(PhotonPipelineResult result)
    {
        if (!result.hasTargets()) {
            return false;
        }

        if (result.getMultiTagResult().isPresent()) {
            return true;
        }

        return result.getBestTarget()
            .getPoseAmbiguity() <= MAX_AMBIGUITY
            && result.getBestTarget()
                .getBestCameraToTarget()
                .getTranslation()
                .getNorm() <= MAX_DISTANCE_METERS;
    }

    public static boolean postFilter(Pose3d pose)
    {
        double x = pose.getX();
        double y = pose.getY();
        double z = pose.getZ();
        return !(z > MAX_Z_METERS
            || x < 0.0 || x > FIELD_LENGTH
            || y < 0.0 || y > FIELD_WIDTH);
    }
}
