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

/**
 * Implementation of {@link VisionIO} for a real PhotonVision camera.
 *
 * <p>
 * This class interfaces with a {@link PhotonCamera} running PhotonVision software to acquire vision
 * data such as fiducial tag detections and estimated poses. It converts raw pipeline results into
 * structured {@link VisionObservation} and {@link TagObservation} objects used by higher-level
 * vision processing systems.
 */
public class VisionIOPhotonVision implements VisionIO {
    protected final PhotonCamera photonCamera;
    private final Camera camera;

    /**
     * Creates a new PhotonVision IO implementation for real hardware.
     *
     * @param camera The {@link Camera} configuration object describing this vision device.
     */
    public VisionIOPhotonVision(Camera camera)
    {
        this.camera = camera;
        this.photonCamera = new PhotonCamera(camera.name());
    }

    /**
     * Converts a PhotonVision pipeline result into a list of {@link TagObservation}s.
     *
     * @param result The {@link PhotonPipelineResult} containing detected fiducial targets.
     * @return An {@link Optional} containing a list of {@link TagObservation}s if targets exist, or
     *         {@link Optional#empty()} if no targets were detected.
     */
    private Optional<List<TagObservation>> tagObservationsFromPipelineResult(
        PhotonPipelineResult result) {
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

    /**
     * Extracts a {@link VisionObservation} from a {@link PhotonPipelineResult}.
     *
     * <p>
     * If a multi-tag pose estimation is available, it will be used; otherwise, the observation will
     * contain only the individual tag detections.
     *
     * @param result The pipeline result from the PhotonVision camera.
     * @return An {@link Optional} containing a valid {@link VisionObservation}, or empty if no
     *         targets were detected.
     */
    private Optional<VisionObservation> poseObservationFromPipelineResult(
        PhotonPipelineResult result) {
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

    /**
     * Updates the {@link VisionIOInputs} structure with the latest data from PhotonVision.
     *
     * <p>
     * This includes connection status, all unread pipeline results, and any successfully derived
     * {@link VisionObservation}s.
     *
     * @param inputs The input data structure to populate with current vision data.
     */
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = photonCamera.isConnected();

        if (!inputs.connected) {
            return;
        }

        inputs.poseObservations = photonCamera.getAllUnreadResults().stream()
            // Convert each unread result to an optional VisionObservation
            .map(this::poseObservationFromPipelineResult)
            // Remove failed conversions
            .filter(Optional::isPresent)
            // Unwrap valid observations
            .map(Optional::get)
            // Collect into an array
            .toArray(VisionObservation[]::new);
    }
}
