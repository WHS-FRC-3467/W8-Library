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
import static edu.wpi.first.units.Units.Seconds;
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
import frc.lib.util.PhotonStrategyHelper;
import frc.lib.util.Timestamped;
import frc.robot.subsystems.vision.VisionConstants.CameraConstants;
import frc.lib.io.vision.VisionIO;
import frc.lib.io.vision.VisionIO.VisionIOInputs;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputs[] inputs;
    private final Alert[] disconnectedAlerts;
    private final CameraConstants[] cameraConstants = VisionConstants.cameraConstants;

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
        this.inputs = new VisionIOInputs[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputs();
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

            // Loop over pose observations
            for (var observation : inputs[cameraIndex].results) {


                var optionalPose = PhotonStrategyHelper.multiTagOnCoprocStrategy(
                    observation,
                    aprilTagLayout,
                    cameraConstants[cameraIndex].robotToCamera());



                if (optionalPose.isEmpty()) {
                    optionalPose = PhotonStrategyHelper.lowestAmbiguityStrategy(
                        observation,
                        aprilTagLayout,
                        cameraConstants[cameraIndex].robotToCamera());
                    if (optionalPose.isEmpty()) {
                        continue;
                    }
                }

                var estimatedPose = optionalPose.get();
                Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/PoseStrategy",
                    estimatedPose.strategy.toString());
                // Add pose to log
                robotPoses.add(estimatedPose.estimatedPose);

                // Add tags used to logged list
                for (PhotonTrackedTarget target : estimatedPose.targetsUsed) {
                    var tagPose = aprilTagLayout.getTagPose(target.getFiducialId());
                    if (tagPose.isPresent()) {
                        tagPoses.add(tagPose.get());
                    }
                }

                // Reject pose if it doesn't pass tuned criteria
                if (shouldRejectPose(estimatedPose.estimatedPose, estimatedPose.targetsUsed.size(),
                    getAvgAmbiguity(estimatedPose.targetsUsed))) {
                    robotPosesRejected.add(estimatedPose.estimatedPose);
                    continue;
                } else {
                    robotPosesAccepted.add(estimatedPose.estimatedPose);
                }

                // Calculate std devs
                double stdDevFactor =
                    (Math.pow(getAvgDistance(estimatedPose.targetsUsed), 2.0)
                        + 10 * estimatedPose.targetsUsed.size())
                        / estimatedPose.targetsUsed.size();

                double linearStdDev = linearStdDevBaseline * stdDevFactor;
                double angularStdDev = angularStdDevBaseline * stdDevFactor;

                // Adjust std devs based on camera trust
                linearStdDev *= cameraConstants[cameraIndex].stdDevFactor();
                angularStdDev *= cameraConstants[cameraIndex].stdDevFactor();


                if (estimatedPose.targetsUsed.size() == 1) {
                    angularStdDev = 1e6;
                }


                // Send vision observation
                consumer.accept(
                    estimatedPose.estimatedPose.toPose2d(),
                    Seconds.of(observation.getTimestampSeconds()),
                    VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
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

    private double getAvgAmbiguity(List<PhotonTrackedTarget> targets)
    {
        double totalAmbiguity = 0.0;
        for (PhotonTrackedTarget target : targets) {
            totalAmbiguity += target.getPoseAmbiguity();
        }
        return totalAmbiguity / targets.size();

    }

    private double getAvgDistance(List<PhotonTrackedTarget> targets)
    {
        double totalDistance = 0.0;
        for (PhotonTrackedTarget target : targets) {
            totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        return totalDistance / targets.size();

    }
}
