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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.devices.AprilTagCamera;
import frc.lib.posestimator.PoseEstimator.VisionPoseObservation;
import frc.lib.posestimator.visionprocessors.LowestAmbiguity;
import frc.lib.posestimator.visionprocessors.MultiTagOnCoproc;
import frc.lib.posestimator.visionprocessors.VisionProcessor.VisionPoseRecord;
import frc.robot.FieldConstants;
import frc.robot.RobotState;

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
     * Default command that replaces the old periodic() behavior.
     */
    private Command visionProcessingCommand()
    {
        return run(() -> {
            for (var camera : cameras) {
                PhotonPipelineResult[] results =
                    camera.getUnreadResults().orElse(null);
                if (results == null) {
                    continue;
                }

                for (var result : results) {
                    if (!preFilter(result)) {
                        continue;
                    }

                    Rotation2d heading = robotState
                        .getPoseAtTime(result.getTimestampSeconds())
                        .map(Pose2d::getRotation)
                        .orElse(null);
                    if (heading == null) {
                        continue;
                    }

                    VisionPoseRecord poseRecord =
                        visionProcessor.processVisionObservation(
                            result,
                            camera.getProperties(),
                            heading).orElse(null);

                    if (poseRecord == null || !postFilter(poseRecord.pose())) {
                        continue;
                    }

                    double stdDevFactor =
                        (Math.pow(poseRecord.averageDistanceMeters(), 2.0)
                            / result.getTargets().size())
                            * camera.getProperties().stdDevFactor();

                    robotState.addVisionObservation(
                        new VisionPoseObservation(
                            result.getTimestampSeconds(),
                            poseRecord.pose().toPose2d(),
                            LINEAR_STDDEV_BASELINE * stdDevFactor,
                            ANGULAR_STDDEV_BASELINE * stdDevFactor));
                }
            }
        });
    }

    /**
     * Creates a command that performs camera extrinsic calibration.
     *
     * @param primaryCameraIndex index into {@code cameras[]} designating the primary camera
     * @param robotCenterTransform known transform from robot origin to robot center
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

                // Estimate primary camera -> robot
                Transform3d primaryCamToRobot =
                    average(samples.get(primary)).plus(robotCenterTransform);

                Logger.recordOutput(
                    "VisionCalibration/PrimaryCameraToRobot",
                    primaryCamToRobot);

                // Other cameras: camera -> primary -> robot
                for (var camera : cameras) {
                    if (camera == primary) {
                        continue;
                    }

                    Transform3d camToPrimary =
                        average(samples.get(camera));

                    Transform3d camToRobot =
                        camToPrimary.plus(primaryCamToRobot);

                    Logger.recordOutput(
                        "VisionCalibration/" + camera.getProperties().name()
                            + "/CameraToRobot",
                        camToRobot);
                }
            }

            /**
             * Simple mean of transforms (translation + rotation). Assumes small rotational
             * variance.
             */
            private Transform3d average(ArrayList<Transform3d> list)
            {
                double x = 0;
                double y = 0;
                double z = 0;
                double rx = 0;
                double ry = 0;
                double rz = 0;

                for (var t : list) {
                    x += t.getX();
                    y += t.getY();
                    z += t.getZ();
                    rx += t.getRotation().getX();
                    ry += t.getRotation().getY();
                    rz += t.getRotation().getZ();
                }

                int n = list.size();
                return new Transform3d(
                    x / n,
                    y / n,
                    z / n,
                    new edu.wpi.first.math.geometry.Rotation3d(
                        rx / n,
                        ry / n,
                        rz / n));
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
