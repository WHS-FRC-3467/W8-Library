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
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.devices.AprilTagCamera;
import frc.lib.posestimator.PoseEstimator.VisionPoseObservation;
import frc.lib.posestimator.visionprocessors.LowestAmbiguity;
import frc.lib.posestimator.visionprocessors.MultiTagOnCoproc;
import frc.lib.posestimator.visionprocessors.VisionProcessor.PoseRecord;
import frc.robot.FieldConstants;
import frc.robot.RobotState;

public class VisionSubsystem extends SubsystemBase {
    public static final String LOG_PREFIX = "VisionProcessor/";

    public static final double LINEAR_STDDEV_BASELINE = 0.4;
    public static final double ANGULAR_STDDEV_BASELINE = 0.4;

    public static final double MAX_Z_METERS = 0.75;
    public static final double FIELD_WIDTH = FieldConstants.FIELD_WIDTH.in(Meters);
    public static final double FIELD_LENGTH = FieldConstants.FIELD_LENGTH.in(Meters);

    public static boolean isValid(PoseRecord poseRecord)
    {
        Pose3d pose = poseRecord.pose();
        double x = pose.getX();
        double y = pose.getY();
        double z = pose.getZ();
        return !(z > MAX_Z_METERS || x < 0.0 || x > FIELD_LENGTH || y < 0.0 || y > FIELD_WIDTH);
    }

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
    }

    /**
     * Processes a {@link PhotonPipelineResult}.
     * 
     * @param result The result to process
     * @param camera The camera the result came from
     * @return Whether or not the result was accepted
     */
    private boolean processPipelineResult(PhotonPipelineResult result, AprilTagCamera camera)
    {
        Rotation2d heading = robotState.getPoseAtTime(result.getTimestampSeconds())
            .map(Pose2d::getRotation).orElse(null);
        if (heading == null) {
            return false;
        }

        PoseRecord poseRecord = visionProcessor.processVisionObservation(
            result,
            camera.getProperties(),
            heading)
            .orElse(null);

        if (poseRecord == null || !isValid(poseRecord)) {
            return false;
        }

        double stdDevFactor =
            (Math.pow(poseRecord.averageDistanceMeters(), 2.0) / result.getTargets().size())
                * VisionConstants.FRONT_LEFT.stdDevFactor();
        double linearStdDev = LINEAR_STDDEV_BASELINE * stdDevFactor;
        double angularStdDev = ANGULAR_STDDEV_BASELINE * stdDevFactor;

        robotState.addVisionObservation(
            new VisionPoseObservation(
                result.getTimestampSeconds(),
                poseRecord.pose().toPose2d(),
                linearStdDev,
                angularStdDev));

        return true;
    }

    @Override
    public void periodic()
    {
        for (var camera : cameras) {
            String cameraLogPrefix = LOG_PREFIX + camera.getProperties().name() + "/";

            PhotonPipelineResult[] results = camera.getUnreadResults().orElse(null);
            if (results == null) {
                continue;
            }

            ArrayList<PhotonPipelineResult> acceptedResults = new ArrayList<>();
            ArrayList<PhotonPipelineResult> rejectedResults = new ArrayList<>();
            for (var result : results) {
                boolean accepted = processPipelineResult(result, camera);

                if (!accepted) {
                    rejectedResults.add(result);
                    continue;
                }

                acceptedResults.add(result);
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
        }
    }
}
