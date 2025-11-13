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

package frc.lib.io.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.Timestamped;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.apache.commons.lang3.ArrayUtils;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
    protected final PhotonCamera camera;
    protected final PhotonPoseEstimator globalPoseEstimator;
    protected final PhotonPoseEstimator trigPoseEstimator;

    private int[] allowedTrigTags = new int[0];

    private final ConstrainedSolvepnpParams constrainedParams =
        new ConstrainedSolvepnpParams(true, 10.0);

    /**
     * Creates a new VisionIOPhotonVision.
     *
     * @param name The configured name of the camera.
     * @param robotToCamera The 3D position of the camera relative to the robot.
     */
    public VisionIOPhotonVision(
        String name,
        Transform3d robotToCamera,
        AprilTagFieldLayout fieldLayout,
        PoseStrategy globalStrategy,
        PoseStrategy globalFallbackStrategy,
        PoseStrategy trigStrategy)
    {
        camera = new PhotonCamera(name);
        globalPoseEstimator = new PhotonPoseEstimator(fieldLayout, globalStrategy, robotToCamera);
        trigPoseEstimator = new PhotonPoseEstimator(fieldLayout, trigStrategy, robotToCamera);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs)
    {
        inputs.connected = camera.isConnected();
        inputs.results = camera.getAllUnreadResults().toArray(new PhotonPipelineResult[0]);


        // List<PoseObservation> estimates = new ArrayList<>();
        // List<PhotonTrackedTarget> allTargets = new ArrayList<>();

        // globalPoseEstimator.addHeadingData(
        // timestampedHeading.timestamp().in(Seconds),
        // timestampedHeading.get());

        // trigPoseEstimator.addHeadingData(
        // timestampedHeading.timestamp().in(Seconds),
        // timestampedHeading.get());

        // for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
        // if (!result.hasTargets()) {
        // continue;
        // }

        // allTargets.addAll(result.getTargets());

        // Optional<EstimatedRobotPose> optionalGlobalEstimate = globalPoseEstimator.update(
        // result,
        // camera.getCameraMatrix(),
        // camera.getDistCoeffs(),
        // Optional.of(constrainedParams));

        // List<PhotonTrackedTarget> singleTargetList = new ArrayList<>();
        // for (PhotonTrackedTarget target : allTargets) {
        // if (ArrayUtils.contains(allowedTrigTags, target.getFiducialId())) {

        // singleTargetList.clear();
        // singleTargetList.add(target);

        // Optional<EstimatedRobotPose> optionalTrigEstimate = trigPoseEstimator.update(
        // new PhotonPipelineResult(
        // result.metadata.sequenceID,
        // result.metadata.captureTimestampMicros,
        // result.metadata.publishTimestampMicros,
        // result.metadata.timeSinceLastPong,
        // singleTargetList),
        // camera.getCameraMatrix(),
        // camera.getDistCoeffs(),
        // Optional.of(constrainedParams));

        // if (optionalTrigEstimate.isEmpty()) {
        // continue;
        // }

        // EstimatedRobotPose trigEstimate = optionalTrigEstimate.get();
        // estimates.add(
        // new PoseObservation(
        // Seconds.of(trigEstimate.timestampSeconds),
        // trigEstimate.estimatedPose,
        // target.poseAmbiguity,
        // 1,
        // Meters.of(target.bestCameraToTarget.getTranslation().getNorm()),
        // true,
        // target.getFiducialId(),
        // trigEstimate.strategy.toString()));
        // }
        // }


        // if (optionalGlobalEstimate.isEmpty()) {
        // continue;
        // }

        // EstimatedRobotPose globalEstimate = optionalGlobalEstimate.get();

        // estimates.add(
        // new PoseObservation(
        // Seconds.of(globalEstimate.timestampSeconds),
        // globalEstimate.estimatedPose,
        // getAvgAmbiguity(globalEstimate.targetsUsed),
        // globalEstimate.targetsUsed.size(),
        // getAvgDistance(globalEstimate.targetsUsed),
        // false,
        // 0,
        // globalEstimate.strategy.toString()));
        // }

        // inputs.poseObservations = estimates.toArray(new PoseObservation[0]);
        // inputs.tagIds = allTargets.stream()
        // .mapToInt(PhotonTrackedTarget::getFiducialId)
        // .distinct()
        // .toArray();

    }

    // // Calculates the average distance of a list of targets.
    // private Distance getAvgDistance(List<PhotonTrackedTarget> targets)
    // {
    // if (targets.size() == 0) {
    // return Meters.zero();
    // }

    // Distance totalDistance = Meters.zero();
    // for (PhotonTrackedTarget target : targets) {
    // totalDistance = totalDistance
    // .plus(Meters.of(target.bestCameraToTarget.getTranslation().getNorm()));
    // }

    // return totalDistance.div(targets.size());
    // }

    // // Calculates the average ambiguity of a list of targets.
    // private double getAvgAmbiguity(List<PhotonTrackedTarget> targets)
    // {
    // if (targets.size() == 0) {
    // return 0.0;
    // }

    // double totalAmbiguity = 0.0;
    // for (PhotonTrackedTarget target : targets) {
    // totalAmbiguity += target.poseAmbiguity;
    // }

    // return totalAmbiguity / targets.size();
    // }

    // // Sets the allowed tags for the trig pose estimator.
    // @Override
    // public void setAllowedTrigTags(int[] trigTags)
    // {
    // allowedTrigTags = trigTags;
    // }


}
