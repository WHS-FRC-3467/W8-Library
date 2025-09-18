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
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.apache.commons.lang3.ArrayUtils;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
    protected final PhotonCamera camera;
    protected final PhotonPoseEstimator poseEstimator;

    /**
     * Creates a new VisionIOPhotonVision.
     *
     * @param name The configured name of the camera.
     * @param robotToCamera The 3D position of the camera relative to the robot.
     */
    public VisionIOPhotonVision(String name, Transform3d robotToCamera,
        AprilTagFieldLayout fieldLayout, PoseStrategy strategy)
    {
        camera = new PhotonCamera(name);
        poseEstimator = new PhotonPoseEstimator(fieldLayout, strategy, robotToCamera);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs, Timestamped<Rotation2d> timestampedHeading)
    {
        inputs.connected = camera.isConnected();

        List<PoseObservation> estimates = new ArrayList<>();
        List<PhotonTrackedTarget> allTargets = new ArrayList<>();
        List<TagObservation> tagObservations = new ArrayList<>();
        Set<Integer> tagIDs = new HashSet<>();

        poseEstimator.addHeadingData(timestampedHeading.timestamp().in(Seconds),
            timestampedHeading.get());
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            if (!result.hasTargets()) {
                continue;
            }

            allTargets.addAll(result.getTargets());

            Optional<EstimatedRobotPose> optionalEstimate = poseEstimator.update(result);
            if (optionalEstimate.isEmpty()) {
                continue;
            }

            EstimatedRobotPose estimate = optionalEstimate.get();

            int tagCount = estimate.targetsUsed.size();

            Distance totalDistance = Meters.zero();
            double totalAmbiguity = 0.0;
            for (PhotonTrackedTarget target : estimate.targetsUsed) {
                totalDistance = totalDistance
                    .plus(Meters.of(target.bestCameraToTarget.getTranslation().getNorm()));
                totalAmbiguity += target.poseAmbiguity;
                tagIDs.add(target.fiducialId);
            }

            Distance averageDistance = totalDistance.div(tagCount);
            double averageAmbiguity = totalAmbiguity / tagCount;
            estimates.add(
                new PoseObservation(
                    Seconds.of(estimate.timestampSeconds),
                    estimate.estimatedPose,
                    averageAmbiguity,
                    tagCount,
                    averageDistance));
        }

        inputs.poseObservations = estimates.toArray(new PoseObservation[0]);
        inputs.tagIds = ArrayUtils.toPrimitive(tagIDs.toArray(new Integer[0]));

        for (PhotonTrackedTarget target : allTargets) {
            tagObservations.add(new TagObservation(
                target.fiducialId,
                target.getPitch(),
                target.getYaw(),
                target.getArea()));
        }
        inputs.allTargets = tagObservations.toArray(new TagObservation[0]);
    }
}
