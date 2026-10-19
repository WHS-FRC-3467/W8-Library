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

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.devices.AprilTagCamera;
import frc.lib.io.vision.VisionIO.CameraResult;
import frc.lib.io.vision.VisionIO.MultiTagObservation;
import frc.lib.io.vision.VisionIO.TagObservation;
import frc.lib.posestimator.PoseEstimator.VisionPoseObservation;
import frc.lib.util.FieldUtil;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.RobotState;

import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

/**
 * The {@code VisionSubsystem} manages all vision-related processing for the robot.
 *
 * <p>It uses one or more {@link AprilTagCamera}s to detect field elements and estimate the robot's
 * pose on the field. Observations are processed through the MultiTagOnCoproc vision processor, with
 * a fallback to LowestAmbiguity if necessary. Valid observations are added to {@link RobotState}
 * for use in localization and navigation.
 *
 * <p>The subsystem periodically polls cameras for new results and logs both accepted and rejected
 * vision observations.
 */
public class VisionSubsystem extends SubsystemBase {

    /** Maximum allowable height (Z-axis) of a detected pose to be considered valid. */
    public static final double MAX_Z_METERS = 0.3;

    /** Maximum allowable distance from a target to be considered valid. */
    public static final double MAX_DISTANCE_METERS = 8.0;

    /** Maximum ambiguity ratio allowed in a result */
    public static final double MAX_AMBIGUITY = 0.2;

    /** Maximum unread results processed per camera cycle. */
    public static final int MAX_UNREAD_RESULTS = 5;

    /** Width of the field in meters. */
    public static final double FIELD_WIDTH = FieldConstants.FIELD_WIDTH;

    /** Length of the field in meters. */
    public static final double FIELD_LENGTH = FieldConstants.FIELD_LENGTH;

    public static final record VisionPoseRecord(
            Pose3d pose, List<Integer> tagsUsed, double averageDistanceMeters) {}

    private static final LoggedTunableNumber TIMESTAMP_OFFSET =
            new LoggedTunableNumber("VisionSubsystem/TimestampOffset", -(1.0 / 45.0));

    private final RobotState robotState = RobotState.getInstance();

    /** How much each accepted pose is trusted. See {@link TrustModel}. */
    private final TrustModel trustModel = TrustModel.recommended();

    private final AprilTagCamera[] cameras;

    /**
     * Quickly checks whether a result is likely to be useful before full processing.
     *
     * <p>Rejects results with no targets, ambiguous poses above 0.2, or targets farther than 4
     * meters.
     *
     * @param result the pipeline result to pre-filter
     * @return {@code true} if the result passes preliminary checks, {@code false} otherwise
     */
    public static boolean preFilter(CameraResult result) {
        // Accept multi-tag results when a multi-tag solution is present and close enough
        if (result.multiTagObservation().isPresent()) {
            return getAvgDistanceMeters(result) < MAX_DISTANCE_METERS;
        }

        // Reject results with no tag observations
        if (result.tagObservations().length == 0) {
            return false;
        }

        // Reject single tag results over MAX_AMBIGUITY or MAX_DISTANCE_METERS
        if (result.tagObservations().length == 1) {
            TagObservation target = result.tagObservations()[0];
            if (target.ambiguity() > MAX_AMBIGUITY) {
                return false;
            }
            if (cameraToTagDistance(target.fiducialId(), target.fieldToCameraPose())
                    > MAX_DISTANCE_METERS) {
                return false;
            }
            return true;
        }

        // Multi-tag frame but no multi-tag solve (e.g., solver disabled/unavailable):
        // fall back to the lowest-ambiguity single tag so usable frames aren't dropped
        TagObservation best =
                Arrays.stream(result.tagObservations())
                        .min(Comparator.comparingDouble(TagObservation::ambiguity))
                        .orElse(null);
        if (best == null) return false;
        if (best.ambiguity() > MAX_AMBIGUITY) return false;
        return cameraToTagDistance(best.fiducialId(), best.fieldToCameraPose())
                <= MAX_DISTANCE_METERS;
    }

    /**
     * Checks whether a given {@link Pose3d} is valid on the field.
     *
     * <p>A pose is considered valid if it is within field boundaries and below {@link
     * #MAX_Z_METERS}.
     *
     * @param pose the pose to validate
     * @return {@code true} if the pose is valid, {@code false} otherwise
     */
    public static boolean postFilter(Pose3d pose) {

        // Reject if pose is too high in Z
        if (pose.getZ() > MAX_Z_METERS) {
            return false;
        }

        // Reject if pose is outside boundary
        if (!FieldUtil.isPoseInField(pose.getTranslation().toTranslation2d(), Meters.zero())) {
            return false;
        }

        return true;
    }

    /**
     * Constructs a new {@code VisionSubsystem} with the specified cameras.
     *
     * @param cameras the cameras to use for vision processing
     */
    public VisionSubsystem(AprilTagCamera... cameras) {
        this.cameras = cameras;
    }

    /**
     * Periodically processes vision results from all cameras. Filters, validates, and adds pose
     * observations to RobotState for localization.
     */
    @Override
    public void periodic() {
        for (int c = 0; c < cameras.length; c++) {
            AprilTagCamera camera = cameras[c];
            String cameraLogPrefix =
                    VisionConstants.NAME + "/" + camera.getProperties().name() + "/";

            CameraResult[] results = camera.getUnreadResults().orElse(null);
            if (results == null) {
                continue;
            }
            if (results.length > MAX_UNREAD_RESULTS) {
                results =
                        Arrays.copyOfRange(
                                results, results.length - MAX_UNREAD_RESULTS, results.length);
            }

            ArrayList<CameraResult> acceptedResults = new ArrayList<>();
            ArrayList<CameraResult> rejectedResults = new ArrayList<>();
            ArrayList<Pose3d> acceptedPoses = new ArrayList<>();
            ArrayList<Pose3d> rejectedPoses = new ArrayList<>();

            for (CameraResult result : results) {

                if (!preFilter(result)) {
                    rejectedResults.add(result);
                    continue;
                }

                // Compute robot pose directly from the standardized CameraResult
                Optional<Pose3d> fieldToRobotPose = computeRobotPose(camera, result);
                if (fieldToRobotPose.isEmpty()) {
                    rejectedResults.add(result);
                    continue;
                }

                List<Integer> tagsUsed = getTagsUsed(result);
                double avgDistanceMeters = getAvgDistanceMeters(result);
                VisionPoseRecord poseRecord =
                        new VisionPoseRecord(fieldToRobotPose.get(), tagsUsed, avgDistanceMeters);

                if (!postFilter(poseRecord.pose())) {
                    rejectedResults.add(result);
                    rejectedPoses.add(poseRecord.pose());
                    continue;
                }

                // captureTimestampUs holds the absolute capture timestamp in microseconds
                double timestampSeconds =
                        result.captureTimestampUs() / 1_000_000.0 + TIMESTAMP_OFFSET.get();
                TrustModel.StdDevs stdDevs =
                        computeStdDevs(camera, result, poseRecord, timestampSeconds);

                robotState.addVisionObservation(
                        new VisionPoseObservation(
                                timestampSeconds,
                                poseRecord.pose().toPose2d(),
                                poseRecord.averageDistanceMeters(),
                                poseRecord.tagsUsed(),
                                stdDevs.xy(),
                                stdDevs.theta()));

                acceptedResults.add(result);
                acceptedPoses.add(poseRecord.pose());
            }

            Set<Integer> tagsAccepted = new HashSet<>();
            Set<Integer> tagsRejected = new HashSet<>();

            Logger.recordOutput(cameraLogPrefix + "Results/AcceptedLength", acceptedResults.size());
            for (CameraResult accepted : acceptedResults) {
                tagsAccepted.addAll(getTagsUsed(accepted));
            }

            Logger.recordOutput(cameraLogPrefix + "Results/RejectedLength", rejectedResults.size());
            for (CameraResult rejected : rejectedResults) {
                tagsRejected.addAll(getTagsUsed(rejected));
            }

            Logger.recordOutput(
                    cameraLogPrefix + "Poses/Accepted", acceptedPoses.toArray(Pose3d[]::new));
            Logger.recordOutput(
                    cameraLogPrefix + "Poses/Rejected", rejectedPoses.toArray(Pose3d[]::new));

            List<Pose3d> tagPosesAccepted =
                    tagsAccepted.stream()
                            .map(this::getTagPose)
                            .filter(Optional::isPresent)
                            .map(Optional::get)
                            .toList();

            List<Pose3d> tagPosesRejected =
                    tagsRejected.stream()
                            .map(this::getTagPose)
                            .filter(Optional::isPresent)
                            .map(Optional::get)
                            .toList();

            Logger.recordOutput(
                    cameraLogPrefix + "TagPoses/Accepted", tagPosesAccepted.toArray(Pose3d[]::new));
            Logger.recordOutput(
                    cameraLogPrefix + "TagPoses/Rejected", tagPosesRejected.toArray(Pose3d[]::new));
        }
    }

    /**
     * Computes the robot pose in field coordinates from a {@link CameraResult}.
     *
     * <p>Prefers the multi-tag observation when available (more accurate). Falls back to the
     * single-tag observation with the lowest ambiguity. The camera-to-robot transform from the
     * camera properties is applied to convert the field-to-camera pose into a field-to-robot pose.
     *
     * @param camera the camera that produced the result
     * @param result the standardized camera result
     * @return the robot pose in field coordinates, or empty if it cannot be computed
     */
    private Optional<Pose3d> computeRobotPose(AprilTagCamera camera, CameraResult result) {
        // The camera-to-robot transform: invert robotToCamera to go camera→robot
        Transform3d cameraToRobot = camera.getProperties().robotToCamera().inverse();

        if (result.multiTagObservation().isPresent()) {
            Pose3d fieldToCamera = result.multiTagObservation().get().fieldToCameraPose();
            return Optional.of(fieldToCamera.transformBy(cameraToRobot));
        }

        if (result.tagObservations().length > 0) {
            // Pick the single tag with the lowest pose ambiguity
            TagObservation best =
                    Arrays.stream(result.tagObservations())
                            .min(Comparator.comparingDouble(TagObservation::ambiguity))
                            .orElse(null);
            if (best == null) return Optional.empty();
            return Optional.of(best.fieldToCameraPose().transformBy(cameraToRobot));
        }

        return Optional.empty();
    }

    /**
     * Returns the distance in meters from the camera to an observed tag.
     *
     * <p>Looks up the tag's known field position from the AprilTag layout and computes the 3D
     * distance between the tag and the camera (both in field coordinates). Falls back to the
     * camera's distance from the field origin if the tag ID is not in the layout.
     */
    private static double cameraToTagDistance(int tagID, Pose3d fieldToCameraPose) {
        Optional<Pose3d> tagPoseOpt = AprilTagLayoutType.NO_TRENCH.getLayout().getTagPose(tagID);
        if (tagPoseOpt.isEmpty()) {
            return fieldToCameraPose.getTranslation().getNorm();
        }
        return tagPoseOpt.get().getTranslation().getDistance(fieldToCameraPose.getTranslation());
    }

    /**
     * Returns the average distance (meters) from the camera to each observed tag.
     *
     * <p>Computes the 3D camera↔tag distance using each tag's known field pose from the AprilTag
     * layout, falling back to the camera's distance from the field origin if a tag is not found.
     */
    private static double getAvgDistanceMeters(CameraResult result) {
        if (result.multiTagObservation().isPresent()) {
            MultiTagObservation obs = result.multiTagObservation().get();
            return Arrays.stream(obs.fiducialIds())
                    .mapToDouble(id -> cameraToTagDistance(id, obs.fieldToCameraPose()))
                    .average()
                    .orElse(0.0);
        }
        if (result.tagObservations().length == 0) return 0.0;
        return Arrays.stream(result.tagObservations())
                .mapToDouble(o -> cameraToTagDistance(o.fiducialId(), o.fieldToCameraPose()))
                .average()
                .orElse(0.0);
    }

    /**
     * Computes how much to trust a vision measurement, using {@link TrustModel}.
     *
     * <p>The model's edge-distance term is skipped because {@link CameraResult} carries no corner
     * data, and so is its reprojection-error term: the multi-tag error is a pixel reprojection
     * error from PhotonVision, but the C2 backend does not document its unit. The decision margin
     * is not available from either camera.
     *
     * @param camera camera used to produce the measurement
     * @param result standardized camera result
     * @param poseRecord estimated pose record
     * @param captureTimestampSeconds when the frame was captured, used to look up how fast the
     *     robot was moving then
     * @return standard deviations for x and y (meters) and heading (radians, infinite when the
     *     heading should be ignored)
     */
    private TrustModel.StdDevs computeStdDevs(
            AprilTagCamera camera,
            CameraResult result,
            VisionPoseRecord poseRecord,
            double captureTimestampSeconds) {
        int tagCount = Math.max(1, poseRecord.tagsUsed().size());

        // Ambiguity only means something for a single tag
        double ambiguity = Double.NaN;
        if (result.multiTagObservation().isEmpty()) {
            double rawAmbiguity =
                    Arrays.stream(result.tagObservations())
                            .mapToDouble(TagObservation::ambiguity)
                            .min()
                            .orElse(1.0);
            ambiguity = Math.max(0.0, rawAmbiguity);
        }

        TrustModel.Observation observation =
                new TrustModel.Observation(
                        tagCount,
                        poseRecord.averageDistanceMeters(),
                        ambiguity,
                        Double.NaN,
                        Double.NaN,
                        Double.NaN);

        ChassisSpeeds velocity = robotState.getRobotRelativeVelocityAtTime(captureTimestampSeconds);
        TrustModel.Motion motion =
                new TrustModel.Motion(
                        Math.hypot(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond),
                        velocity.omegaRadiansPerSecond);

        return trustModel.stdDevs(observation, motion, camera.getProperties().stdDevFactor());
    }

    /**
     * Returns the list of fiducial IDs observed in a {@link CameraResult}.
     *
     * <p>Uses the multi-tag observation's ID list when available, otherwise collects IDs from
     * individual tag observations.
     */
    private List<Integer> getTagsUsed(CameraResult result) {
        if (result.multiTagObservation().isPresent()) {
            int[] ids = result.multiTagObservation().get().fiducialIds();
            List<Integer> tagList = new ArrayList<>(ids.length);
            for (int id : ids) tagList.add(id);
            return tagList;
        }
        List<Integer> tagList = new ArrayList<>(result.tagObservations().length);
        for (TagObservation obs : result.tagObservations()) {
            tagList.add(obs.fiducialId());
        }
        return tagList;
    }

    private Optional<Pose3d> getTagPose(int id) {
        return AprilTagLayoutType.NO_TRENCH.getLayout().getTagPose(id);
    }
}
