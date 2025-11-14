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
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Timestamped;
import frc.lib.PhotonPoseEstimatorPlus;
import frc.lib.PhotonPoseEstimatorPlus.PoseStrategy;
import frc.lib.io.vision.VisionIO;
import frc.lib.io.vision.VisionIO.VisionIOInputs;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;

public class Vision extends SubsystemBase {
    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputs[] inputs;
    private final Alert[] disconnectedAlerts;
    private final CameraConstants[] cameraConstants = VisionConstants.cameraConstants;
    private final PhotonPoseEstimatorPlus[] poseEstimators;

    private final Supplier<Timestamped<Rotation2d>> timestampedHeadingSupplier;

    public Vision(
        VisionConsumer consumer,
        Supplier<Timestamped<Rotation2d>> timestampedHeadingSupplier,
        VisionIO... io)
    {
        this.consumer = consumer;
        this.timestampedHeadingSupplier = timestampedHeadingSupplier;
        this.io = io;


        // Initialize inputs
        this.poseEstimators = new PhotonPoseEstimatorPlus[io.length];
        this.inputs = new VisionIOInputs[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputs();
            poseEstimators[i] = new PhotonPoseEstimatorPlus(
                aprilTagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cameraConstants[i].robotToCamera());
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] = new Alert(
                "Vision camera " + cameraConstants[i].name() + " is disconnected.",
                AlertType.kWarning);
        }
    }

    @Override
    public void periodic()
    {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera " + cameraConstants[i].name(), inputs[i]);
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new ArrayList<>();
        List<Pose3d> allRobotPoses = new ArrayList<>();
        List<Pose3d> allRobotPosesAccepted = new ArrayList<>();
        List<Pose3d> allRobotPosesRejected = new ArrayList<>();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new ArrayList<>();
            List<Pose3d> robotPoses = new ArrayList<>();
            List<Pose3d> robotPosesAccepted = new ArrayList<>();
            List<Pose3d> robotPosesRejected = new ArrayList<>();

            // Add tag poses
            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : inputs[cameraIndex].results) {
                // Check whether to reject pose

                var optionalPose = poseEstimators[cameraIndex].update(observation);

                if (optionalPose.isEmpty()) {
                    continue;
                }
                var estimatedPose = optionalPose.get();
                // Add pose to log
                robotPoses.add(estimatedPose.estimatedPose);

                if (shouldRejectPose(estimatedPose.estimatedPose, observation.getTargets().size(),
                    observation.ambiguity)) {
                    (observation.usedTrigEstimator() ? robotTrigPosesRejected : robotPosesRejected)
                        .add(pose);
                    continue;
                } else {
                    (observation.usedTrigEstimator() ? robotTrigPosesAccepted : robotPosesAccepted)
                        .add(pose);
                }

                // Calculate standard deviations
                if (!observation.usedTrigEstimator()) {
                    double stdDevFactor =
                        (Math.pow(observation.averageTagDistance().in(Meters), 2.0)
                            + 10 * observation.ambiguity())
                            / observation.tagCount();

                    double linearStdDev = linearStdDevBaseline * stdDevFactor;
                    double angularStdDev = angularStdDevBaseline * stdDevFactor;

                    if (observation.tagCount() == 1) {
                        angularStdDev = 1e6;
                    }
                    // if (cameraIndex < cameraStdDevFactors.length) {
                    // linearStdDev *= cameraStdDevFactors[cameraIndex];
                    // angularStdDev *= cameraStdDevFactors[cameraIndex];
                    // }

                    // Send vision observation
                    consumer.accept(
                        pose.toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
                } else {
                    // TODO: add method for trig pose estimator
                }
            }

            // Log camera datadata
            Logger.recordOutput(
                "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
                tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
                robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
                robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
                robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));

            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput(
            "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput(
            "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput(
            "Vision/Summary/RobotPosesAccepted",
            allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput(
            "Vision/Summary/RobotPosesRejected",
            allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(
            Pose2d visionRobotPoseMeters,
            Time timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs);
    }

    private boolean shouldRejectPose(Pose3d pose, int tagCount, double ambiguity)
    {
        boolean rejectPose =
            (tagCount == 1 && ambiguity > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(pose.getZ()) > maxZError // Must have realistic Z
                // Must be within the field boundaries
                || pose.getX() < 0.0
                || pose.getX() > aprilTagLayout.getFieldLength()
                || pose.getY() < 0.0
                || pose.getY() > aprilTagLayout.getFieldWidth();

        return rejectPose;

    }

    private double getAvgAmbiguity()
}
