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
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    /** Baseline linear standard deviation used for vision observations. */
    public static final double LINEAR_STDDEV_BASELINE = 0.4;

    /** Baseline angular standard deviation used for vision observations. */
    public static final double ANGULAR_STDDEV_BASELINE = 0.4;

    /** Maximum allowable height (Z-axis) of a detected pose to be considered valid. */
    public static final double MAX_Z_METERS = 0.75;

    /** Width of the field in meters. */
    public static final double FIELD_WIDTH = FieldConstants.FIELD_WIDTH.in(Meters);

    /** Length of the field in meters. */
    public static final double FIELD_LENGTH = FieldConstants.FIELD_LENGTH.in(Meters);

    /**
     * Quickly checks whether a {@link PhotonPipelineResult} is likely to be useful before full
     * processing.
     * <p>
     * Rejects results with no targets, ambiguous poses above 0.2, or targets farther than 10
     * meters.
     * </p>
     *
     * @param result the pipeline result to pre-filter
     * @return {@code true} if the result passes preliminary checks, {@code false} otherwise
     */
    public static boolean preFilter(PhotonPipelineResult result)
    {
        if (!result.hasTargets()) {
            return false;
        }

        if (result.getMultiTagResult().isPresent()) {
            return true;
        }

        PhotonTrackedTarget bestTarget = result.getBestTarget();

        if (bestTarget.getBestCameraToTarget().getTranslation().getNorm() > 10.0) {
            return false;
        }

        return bestTarget.getPoseAmbiguity() <= 0.2;
    }

    /**
     * Checks whether a given {@link VisionPoseRecord} is valid on the field.
     * <p>
     * A pose is considered valid if it is within field boundaries and below {@link #MAX_Z_METERS}.
     * </p>
     *
     * @param poseRecord the pose record to validate
     * @return {@code true} if the pose is valid, {@code false} otherwise
     */
    public static boolean postFilter(VisionPoseRecord poseRecord)
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

    /**
     * Constructs a new {@code VisionSubsystem} with the specified cameras.
     *
     * @param cameras the cameras to use for vision processing
     */
    public VisionSubsystem(AprilTagCamera... cameras)
    {
        this.cameras = cameras;
    }

    /**
     * Processes a single {@link PhotonPipelineResult} from a camera.
     * <p>
     * If the result is valid and passes checks, it is converted to a {@link VisionPoseObservation}
     * and added to {@link RobotState}. The method also computes standard deviation factors for
     * linear and angular uncertainty.
     * </p>
     *
     * @param result the pipeline result to process
     * @param camera the camera that generated the result
     * @return {@code true} if the result was accepted and processed, {@code false} otherwise
     */
    private boolean processPipelineResult(PhotonPipelineResult result, AprilTagCamera camera)
    {
        if (!preFilter(result)) {
            return false;
        }

        Rotation2d heading = robotState.getPoseAtTime(result.getTimestampSeconds())
            .map(Pose2d::getRotation).orElse(null);
        if (heading == null) {
            return false;
        }

        VisionPoseRecord poseRecord = visionProcessor.processVisionObservation(
            result,
            camera.getProperties(),
            heading)
            .orElse(null);

        if (poseRecord == null || !postFilter(poseRecord)) {
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

    /**
     * Periodically called by the scheduler.
     * <p>
     * This method polls all cameras for unread results, processes each result using
     * {@link #processPipelineResult(PhotonPipelineResult, AprilTagCamera)}, and logs accepted and
     * rejected observations for debugging and analysis.
     * </p>
     */
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
