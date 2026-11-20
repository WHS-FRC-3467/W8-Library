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

import frc.lib.io.vision.VisionIO.CameraResult;
import frc.lib.sensors.AprilTagCamera.CameraProperties;

import org.photonvision.PhotonCamera;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.wpilib.fields.Field;
import org.wpilib.math.geometry.Pose3d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;

/**
 * Real hardware implementation of VisionIO using PhotonVision.
 *
 * <p>Connects to a PhotonVision coprocessor running an AprilTag detection pipeline and reads vision
 * results over NetworkTables. Used for real robot operation.
 *
 * <p>{@link #updateInputs} writes raw packed bytes into the logged {@link VisionIOInputs} for
 * AdvantageKit replay compatibility. {@link #decodeResults} then deserializes those bytes and
 * converts them to standardized {@link CameraResult} records.
 */
public class VisionIOPhotonVision implements VisionIO {
    protected final PhotonCamera photonCamera;

    /**
     * Constructs a PhotonVision camera interface.
     *
     * @param cameraProperties Camera configuration including name and calibration
     */
    public VisionIOPhotonVision(CameraProperties cameraProperties) {
        this.photonCamera = new PhotonCamera(cameraProperties.name());
    }

    /**
     * Reads all unread results from PhotonVision, packs them as raw bytes with the magic prefix,
     * and stores them in {@code inputs} for AdvantageKit logging and replay.
     */
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = photonCamera.isConnected();
        inputs.rawPacketType = NativePacketType.PHOTON;

        if (!inputs.connected) {
            inputs.rawResults = new byte[0][];
            inputs.captureTimestampsUs = new long[0];
            inputs.publishTimestampsUs = new long[0];
            return;
        }

        var unreadResults = photonCamera.getAllUnreadResults();
        int n = unreadResults.size();
        inputs.rawResults = new byte[n][];
        inputs.captureTimestampsUs = new long[n];
        inputs.publishTimestampsUs = new long[n];
        for (int i = 0; i < n; i++) {
            var result = unreadResults.get(i);
            inputs.rawResults[i] = packPhotonResult(result);
            inputs.captureTimestampsUs[i] = result.metadata.captureTimestampNanos / 1000;
            inputs.publishTimestampsUs[i] = result.metadata.publishTimestampNanos / 1000;
        }
    }

    /**
     * Decodes the raw PhotonVision bytes stored in {@code inputs} into {@link CameraResult}
     * records.
     *
     * <p>Each raw byte array is unpacked from the PhotonVision struct format, then each tracked
     * target's field-to-camera pose is reconstructed using the known tag field positions.
     */
    public static CameraResult[] decodeResults(VisionIOInputs inputs, Field tagLayout) {
        ArrayList<CameraResult> results = new ArrayList<>(inputs.rawResults.length);
        for (int i = 0; i < inputs.rawResults.length; i++) {
            byte[] raw = inputs.rawResults[i];
            if (raw == null || raw.length == 0) continue;

            PhotonPipelineResult photon = unpackPhotonResult(raw);
            if (photon == null) continue;

            long captureTs =
                    i < inputs.captureTimestampsUs.length ? inputs.captureTimestampsUs[i] : 0;
            long publishTs =
                    i < inputs.publishTimestampsUs.length ? inputs.publishTimestampsUs[i] : 0;

            results.add(toCameraResult(photon, captureTs, publishTs, tagLayout));
        }
        return results.toArray(CameraResult[]::new);
    }

    // -------------------------------------------------------------------------
    // Internal helpers
    // -------------------------------------------------------------------------

    private static PhotonPipelineResult unpackPhotonResult(byte[] raw) {
        try {
            byte[] payload = Arrays.copyOf(raw, raw.length);
            return PhotonPipelineResult.photonStruct.unpack(new Packet(payload));
        } catch (RuntimeException e) {
            return null;
        }
    }

    /**
     * Converts a deserialized {@link PhotonPipelineResult} to a standardized {@link CameraResult}.
     *
     * <p>For each tracked target, the field-to-camera pose is reconstructed as: {@code
     * fieldToCamera = fieldToTag * inverse(cameraToTag)}.
     */
    private static CameraResult toCameraResult(
            PhotonPipelineResult photon,
            long captureTimestampUs,
            long publishTimestampUs,
            Field tagLayout) {
        ArrayList<TagObservation> tagObs = new ArrayList<>(photon.getTargets().size());
        for (PhotonTrackedTarget target : photon.getTargets()) {
            int tagId = target.getFiducialId();
            Optional<Pose3d> tagPoseOpt = tagLayout.getTagPose(tagId);
            if (tagPoseOpt.isEmpty()) continue;
            Pose3d tagPose = tagPoseOpt.get();

            // fieldToCamera = fieldToTag * (cameraToTag)^-1
            Pose3d fieldToCamera = tagPose.transformBy(target.getBestCameraToTarget().inverse());
            Pose3d altFieldToCamera =
                    tagPose.transformBy(target.getAlternateCameraToTarget().inverse());

            tagObs.add(
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
                                mt -> {
                                    Pose3d fieldToCamPose =
                                            new Pose3d(
                                                    mt.estimatedPose.best.getTranslation(),
                                                    mt.estimatedPose.best.getRotation());
                                    int[] ids =
                                            mt.fiducialIDsUsed.stream()
                                                    .mapToInt(Short::intValue)
                                                    .toArray();
                                    return new MultiTagObservation(
                                            ids, fieldToCamPose, mt.estimatedPose.bestReprojErr);
                                });

        return new CameraResult(
                tagObs.toArray(TagObservation[]::new),
                multiTag,
                (double) captureTimestampUs,
                (double) publishTimestampUs);
    }

    private static byte[] packPhotonResult(PhotonPipelineResult result) {
        Packet packet = new Packet(512);
        PhotonPipelineResult.photonStruct.pack(packet, result);
        byte[] raw = packet.getWrittenDataCopy();
        return raw;
    }
}
