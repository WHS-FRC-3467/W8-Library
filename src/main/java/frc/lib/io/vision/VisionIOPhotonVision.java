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
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.Timestamped;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.apache.commons.lang3.ArrayUtils;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.jni.ConstrainedSolvepnpJni;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
    protected final PhotonCamera camera;
    protected final PhotonPoseEstimator poseEstimator;
    private final Optional<ConstrainedSolvepnpParams> constrainedPnpParams;

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
        constrainedPnpParams = Optional.of(new ConstrainedSolvepnpParams(true, 10.0));
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


        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            if (!result.hasTargets()) {
                continue;
            }

            allTargets.addAll(result.getTargets());

            poseEstimator.addHeadingData(timestampedHeading.timestamp().in(Seconds),
                timestampedHeading.get());
            Optional<EstimatedRobotPose> optionalEstimate =
                poseEstimator.update(result, Optional.of(MatBuilder.fill(Nat.N3(), Nat.N3(),
                    // Intrinsic and distort from SimCameraProperties.LL2_1280_720()
                    // intrinsic
                    1011.3749416937393,
                    0.0,
                    645.4955139388737,
                    0.0,
                    1008.5391755084075,
                    508.32877656020196,
                    0.0,
                    0.0,
                    1.0)), Optional.of(
                        VecBuilder.fill( // distort
                            0.13730101577061535,
                            -0.2904345656989261,
                            8.32475714507539E-4,
                            -3.694397782014239E-4,
                            0.09487962227027584,
                            0,
                            0,
                            0)),
                    constrainedPnpParams);

            if (optionalEstimate.isEmpty()) {
                continue;
            }

            EstimatedRobotPose estimate = optionalEstimate.get();
            Logger.recordOutput("Vision/Strategy", estimate.strategy.toString());
            Logger.recordOutput("Vision/Odom Heading", timestampedHeading.get());
            Logger.recordOutput("Vision/estimate heading", estimate.estimatedPose.getRotation());

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
