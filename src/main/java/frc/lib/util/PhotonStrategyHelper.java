// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;

/** Add your docs here. */
public class PhotonStrategyHelper {

    /**
     * Return the estimated position of the robot with the lowest position ambiguity from a List of
     * pipeline results.
     *
     * @param result pipeline result
     * @return the estimated position of the robot in the FCS and the estimated timestamp of this
     *         estimation.
     */
    public static Optional<EstimatedRobotPose> lowestAmbiguityStrategy(
        PhotonPipelineResult result,
        AprilTagFieldLayout fieldTags,
        Transform3d robotToCamera)
    {
        PhotonTrackedTarget lowestAmbiguityTarget = null;

        double lowestAmbiguityScore = 10;

        for (PhotonTrackedTarget target : result.targets) {
            double targetPoseAmbiguity = target.getPoseAmbiguity();
            // Make sure the target is a Fiducial target.
            if (targetPoseAmbiguity != -1 && targetPoseAmbiguity < lowestAmbiguityScore) {
                lowestAmbiguityScore = targetPoseAmbiguity;
                lowestAmbiguityTarget = target;
            }
        }

        // Although there are confirmed to be targets, none of them may be fiducial
        // targets.
        if (lowestAmbiguityTarget == null)
            return Optional.empty();

        int targetFiducialId = lowestAmbiguityTarget.getFiducialId();

        Optional<Pose3d> targetPosition = fieldTags.getTagPose(targetFiducialId);

        if (targetPosition.isEmpty()) {
            // reportFiducialPoseError(targetFiducialId);
            return Optional.empty();
        }

        return Optional.of(
            new EstimatedRobotPose(
                targetPosition
                    .get()
                    .transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
                    .transformBy(robotToCamera.inverse()),
                result.getTimestampSeconds(),
                result.getTargets(),
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY));
    }

    public static Optional<EstimatedRobotPose> multiTagOnCoprocStrategy(
        PhotonPipelineResult result,
        AprilTagFieldLayout fieldTags,
        Transform3d robotToCamera)
    {

        if (result.getMultiTagResult().isEmpty()) {
            return Optional.empty();
        }

        var best_tf = result.getMultiTagResult().get().estimatedPose.best;
        var best =
            Pose3d.kZero
                .plus(best_tf) // field-to-camera
                .relativeTo(fieldTags.getOrigin())
                .plus(robotToCamera.inverse()); // field-to-robot
        return Optional.of(
            new EstimatedRobotPose(
                best,
                result.getTimestampSeconds(),
                result.getTargets(),
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR));
    }

    public static Optional<EstimatedRobotPose> pnpDistanceTrigSolveStrategy(
        PhotonPipelineResult result,
        AprilTagFieldLayout fieldTags,
        Transform3d robotToCamera,
        Rotation2d headingAtObservationTime)
    {
        PhotonTrackedTarget bestTarget = result.getBestTarget();

        if (bestTarget == null)
            return Optional.empty();

        // var headingSampleOpt = headingBuffer.getSample(result.getTimestampSeconds());
        // if (headingSampleOpt.isEmpty()) {
        // return Optional.empty();
        // }
        // Rotation2d headingSample = headingSampleOpt.get();
        var headingSample = headingAtObservationTime;

        Translation2d camToTagTranslation =
            new Translation3d(
                bestTarget.getBestCameraToTarget().getTranslation().getNorm(),
                new Rotation3d(
                    0,
                    -Math.toRadians(bestTarget.getPitch()),
                    -Math.toRadians(bestTarget.getYaw())))
                        .rotateBy(robotToCamera.getRotation())
                        .toTranslation2d()
                        .rotateBy(headingSample);

        var tagPoseOpt = fieldTags.getTagPose(bestTarget.getFiducialId());
        if (tagPoseOpt.isEmpty()) {
            return Optional.empty();
        }
        var tagPose2d = tagPoseOpt.get().toPose2d();

        Translation2d fieldToCameraTranslation =
            tagPose2d.getTranslation().plus(camToTagTranslation.unaryMinus());

        Translation2d camToRobotTranslation =
            robotToCamera.getTranslation().toTranslation2d().unaryMinus().rotateBy(headingSample);

        Pose2d robotPose =
            new Pose2d(fieldToCameraTranslation.plus(camToRobotTranslation), headingSample);

        return Optional.of(
            new EstimatedRobotPose(
                new Pose3d(robotPose),
                result.getTimestampSeconds(),
                result.getTargets(),
                PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE));
    }

    public static Optional<EstimatedRobotPose> constrainedPnpStrategy(
        PhotonPipelineResult result,
        Optional<Matrix<N3, N3>> cameraMatrixOpt,
        Optional<Matrix<N8, N1>> distCoeffsOpt,
        boolean headingFree,
        double headingScaleFactor,
        Rotation2d headingAtObservationTime,
        Pose3d fieldToRobotSeed,
        AprilTagFieldLayout fieldTags,
        Transform3d robotToCamera)
    {

        boolean hasCalibData = cameraMatrixOpt.isPresent() && distCoeffsOpt.isPresent();
        // cannot run multitagPNP, use fallback strategy
        if (!hasCalibData) {
            // return update(
            // result, cameraMatrixOpt, distCoeffsOpt, Optional.empty(),
            // this.multiTagFallbackStrategy);
            return Optional.empty();
        }

        // if (constrainedPnpParams.isEmpty()) {
        // return Optional.empty();
        // }

        // Need heading if heading fixed
        // if (!constrainedPnpParams.get().headingFree
        // && headingBuffer.getSample(result.getTimestampSeconds()).isEmpty()) {
        // return update(
        // result, cameraMatrixOpt, distCoeffsOpt, Optional.empty(),
        // this.multiTagFallbackStrategy);
        // }

        // Pose3d fieldToRobotSeed;

        // // Attempt to use multi-tag to get a pose estimate seed
        // if (result.getMultiTagResult().isPresent()) {
        // fieldToRobotSeed =
        // Pose3d.kZero.plus(
        // result.getMultiTagResult().get().estimatedPose.best
        // .plus(robotToCamera.inverse()));
        // } else {
        // // HACK - use fallback strategy to gimme a seed pose
        // // TODO - make sure nested update doesn't break state
        // var nestedUpdate =
        // update(
        // result,
        // cameraMatrixOpt,
        // distCoeffsOpt,
        // Optional.empty(),
        // this.multiTagFallbackStrategy);
        // if (nestedUpdate.isEmpty()) {
        // // best i can do is bail
        // return Optional.empty();
        // }
        // fieldToRobotSeed = nestedUpdate.get().estimatedPose;
        // }

        // if (!constrainedPnpParams.get().headingFree) {
        if (!headingFree) {
            // If heading fixed, force rotation component
            fieldToRobotSeed =
                new Pose3d(
                    fieldToRobotSeed.getTranslation(),
                    // new Rotation3d(headingBuffer.getSample(result.getTimestampSeconds()).get()));
                    new Rotation3d(headingAtObservationTime));
        }

        var pnpResult =
            VisionEstimation.estimateRobotPoseConstrainedSolvepnp(
                cameraMatrixOpt.get(),
                distCoeffsOpt.get(),
                result.getTargets(),
                robotToCamera,
                fieldToRobotSeed,
                fieldTags,
                // tagModel,
                TargetModel.kAprilTag36h11,
                // constrainedPnpParams.get().headingFree,
                // headingBuffer.getSample(result.getTimestampSeconds()).get(),
                // constrainedPnpParams.get().headingScaleFactor
                headingFree,
                headingAtObservationTime,
                headingScaleFactor);

        // try fallback strategy if solvePNP fails for some reason
        if (!pnpResult.isPresent())
            // return update(
            // result, cameraMatrixOpt, distCoeffsOpt, Optional.empty(),
            // this.multiTagFallbackStrategy);
            return Optional.empty();

        var best = Pose3d.kZero.plus(pnpResult.get().best); // field-to-robot

        return Optional.of(
            new EstimatedRobotPose(
                best,
                result.getTimestampSeconds(),
                result.getTargets(),
                PhotonPoseEstimator.PoseStrategy.CONSTRAINED_SOLVEPNP));
    }

    public static Optional<EstimatedRobotPose> multiTagOnRioStrategy(
        PhotonPipelineResult result,
        Optional<Matrix<N3, N3>> cameraMatrixOpt,
        Optional<Matrix<N8, N1>> distCoeffsOpt,
        AprilTagFieldLayout fieldTags,
        Transform3d robotToCamera)
    {

        // if (cameraMatrixOpt.isEmpty() || distCoeffsOpt.isEmpty()) {
        // DriverStation.reportWarning(
        // "No camera calibration data provided for multi-tag-on-rio",
        // Thread.currentThread().getStackTrace());
        // return update(result, this.multiTagFallbackStrategy);
        // }

        // if (result.getTargets().size() < 2) {
        // return update(result, this.multiTagFallbackStrategy);
        // }

        var pnpResult =
            VisionEstimation.estimateCamPosePNP(
                cameraMatrixOpt.get(), distCoeffsOpt.get(), result.getTargets(), fieldTags,
                TargetModel.kAprilTag36h11);
        // try fallback strategy if solvePNP fails for some reason
        if (!pnpResult.isPresent())
            // return update(
            // result, cameraMatrixOpt, distCoeffsOpt, Optional.empty(),
            // this.multiTagFallbackStrategy);
            return Optional.empty();

        var best =
            Pose3d.kZero
                .plus(pnpResult.get().best) // field-to-camera
                .plus(robotToCamera.inverse()); // field-to-robot

        return Optional.of(
            new EstimatedRobotPose(
                best,
                result.getTimestampSeconds(),
                result.getTargets(),
                PoseStrategy.MULTI_TAG_PNP_ON_RIO));
    }
}
