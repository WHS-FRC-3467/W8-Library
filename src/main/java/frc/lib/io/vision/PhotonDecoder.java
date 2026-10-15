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
package frc.lib.io.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;

import frc.lib.io.vision.VisionIO.CameraResult;
import frc.lib.io.vision.VisionIO.MultiTagObservation;
import frc.lib.io.vision.VisionIO.TagObservation;

import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.Optional;

/** Decodes PhotonVision results, packed by {@link VisionIOPhotonVision#pack}. */
public class PhotonDecoder {

    private final AprilTagFieldLayout tagLayout;

    /**
     * @param tagLayout field layout, to place each observed tag
     */
    public PhotonDecoder(AprilTagFieldLayout tagLayout) {
        this.tagLayout = tagLayout;
    }

    /**
     * Decodes one result.
     *
     * @param raw the raw bytes
     * @param captureTimestampUs capture time, robot clock (FPGA), microseconds
     * @param publishTimestampUs publish/receive time, robot clock, microseconds
     * @return the result, or {@code null} if {@code raw} isn't valid
     */
    public CameraResult decode(byte[] raw, long captureTimestampUs, long publishTimestampUs) {
        if (raw == null || raw.length == 0) {
            return null;
        }
        PhotonPipelineResult photon;
        try {
            photon = PhotonPipelineResult.photonStruct.unpack(new Packet(raw.clone()));
        } catch (RuntimeException e) {
            return null;
        }

        ArrayList<TagObservation> tagObservations = new ArrayList<>(photon.getTargets().size());
        for (PhotonTrackedTarget target : photon.getTargets()) {
            int tagId = target.getFiducialId();
            Optional<Pose3d> tagPose = tagLayout.getTagPose(tagId);
            if (tagPose.isEmpty()) continue;
            // fieldToCamera = fieldToTag * (cameraToTag)^-1
            Pose3d fieldToCamera =
                    tagPose.get().transformBy(target.getBestCameraToTarget().inverse());
            Pose3d altFieldToCamera =
                    tagPose.get().transformBy(target.getAlternateCameraToTarget().inverse());
            tagObservations.add(
                    new TagObservation(
                            tagId,
                            fieldToCamera,
                            Optional.of(altFieldToCamera),
                            target.getArea(),
                            target.getPoseAmbiguity()));
        }

        Optional<MultiTagObservation> multiTag =
                photon.getMultiTagResult()
                        .map(
                                mt ->
                                        new MultiTagObservation(
                                                mt.fiducialIDsUsed.stream()
                                                        .mapToInt(Short::intValue)
                                                        .toArray(),
                                                new Pose3d(
                                                        mt.estimatedPose.best.getTranslation(),
                                                        mt.estimatedPose.best.getRotation()),
                                                mt.estimatedPose.bestReprojErr));

        return new CameraResult(
                tagObservations.toArray(TagObservation[]::new),
                multiTag,
                (double) captureTimestampUs,
                (double) publishTimestampUs);
    }
}
