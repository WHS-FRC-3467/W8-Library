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

import edu.wpi.first.units.measure.Time;
import frc.lib.util.GeomUtil;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
    protected final PhotonCamera photonCamera;
    private final Camera camera;

    /**
     * Creates a new VisionIOPhotonVision.
     *
     * @param name The name of the camera,
     * @param robotToCamera The transform from the robot to the camera
     */
    public VisionIOPhotonVision(
        Camera camera)
    {
        this.camera = camera;
        this.photonCamera = new PhotonCamera(camera.name());
    }

    private Optional<List<TagObservation>> tagObservationsFromPipelineResult(
        PhotonPipelineResult result)
    {
        if (!result.hasTargets())
            return Optional.empty();

        List<TagObservation> observations = new ArrayList<>();
        result.targets.forEach(target -> {
            var observation = new TagObservation(
                target.fiducialId,
                target.area,
                Degrees.of(target.pitch),
                Degrees.of(target.yaw),
                target.detectedCorners,
                target.bestCameraToTarget,
                target.poseAmbiguity);

            observations.add(observation);
        });
        return Optional.of(observations);
    }

    private Optional<VisionObservation> poseObservationFromPipelineResult(
        PhotonPipelineResult result)
    {
        if (!result.hasTargets())
            return Optional.empty();

        Time timestamp = Seconds.of(result.getTimestampSeconds());

        var optionalTagObservations = tagObservationsFromPipelineResult(result);
        if (optionalTagObservations.isEmpty()) {
            return Optional.empty();
        }
        List<TagObservation> tagObservations = optionalTagObservations.get();

        var multiTagResult = result.getMultiTagResult().map(multiTag -> new VisionObservation(
            timestamp,
            camera,
            Optional.of(GeomUtil.toPose3d(multiTag.estimatedPose.best)),
            tagObservations));

        if (multiTagResult.isPresent()) {
            return Optional.of(multiTagResult.get());
        }

        var singleTagResult = new VisionObservation(
            timestamp,
            camera,
            Optional.empty(),
            tagObservations);

        return Optional.of(singleTagResult);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs)
    {
        inputs.connected = photonCamera.isConnected();

        if (!inputs.connected) {
            return;
        }

        inputs.poseObservations = photonCamera.getAllUnreadResults().stream()
            // For each result, attempt to get a VisionObservation
            .map(this::poseObservationFromPipelineResult)
            // Remove failed attempts
            .filter(Optional::isPresent)
            // All values are present, so we can safelt unwrap the remaining observations
            .map(Optional::get)
            // Convert the stream into an array
            .toArray(l -> new VisionObservation[l]);
    }
}
