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

import frc.lib.devices.AprilTagCamera.CameraProperties;
import frc.lib.io.vision.VisionIO.CameraResult;

import org.photonvision.PhotonCamera;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

/**
 * Real hardware implementation of {@link VisionIO} using PhotonVision.
 *
 * <p>Connects to a PhotonVision coprocessor running an AprilTag detection pipeline and reads vision
 * results over NetworkTables. Used for real robot operation.
 *
 * <p>{@link #updateInputs} writes raw packed bytes into the logged {@link VisionIOInputs} for
 * AdvantageKit replay compatibility. {@link #decodeResults} then deserializes those bytes and
 * converts them to standardized {@link CameraResult} records.
 */
public class VisionIOPhotonVision implements VisionIO {
    // Four-byte camera index followed by the PhotonPipelineResult struct.
    private static final int PACKET_HEADER_BYTES = Integer.BYTES;

    private record CameraSource(CameraProperties properties, PhotonCamera camera) {}

    private record DecodedPhotonPacket(PhotonPipelineResult result, int cameraIndex) {}

    private final List<CameraSource> cameras;

    /**
     * Constructs a PhotonVision camera interface.
     *
     * @param cameraProperties configuration for every physical camera owned by this source
     */
    public VisionIOPhotonVision(CameraProperties[] cameraProperties) {
        if (cameraProperties == null || cameraProperties.length == 0) {
            throw new IllegalArgumentException("At least one camera is required");
        }
        this.cameras =
                Arrays.stream(cameraProperties)
                        .map(
                                properties ->
                                        new CameraSource(
                                                properties, new PhotonCamera(properties.name())))
                        .toList();
    }

    /**
     * Reads all unread results from every connected PhotonVision camera and stores each result as
     * a self-identifying packet for AdvantageKit logging and replay.
     */
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = cameras.stream().allMatch(source -> source.camera().isConnected());
        inputs.rawPacketType = NativePacketType.PHOTON;

        ArrayList<byte[]> rawResults = new ArrayList<>();
        ArrayList<Long> captureTimestampsUs = new ArrayList<>();
        ArrayList<Long> publishTimestampsUs = new ArrayList<>();
        for (CameraSource source : cameras) {
            if (!source.camera().isConnected()) continue;
            for (PhotonPipelineResult result : source.camera().getAllUnreadResults()) {
                rawResults.add(packPhotonResult(result, source.properties().index()));
                captureTimestampsUs.add(result.metadata.captureTimestampMicros);
                publishTimestampsUs.add(result.metadata.publishTimestampMicros);
            }
        }
        inputs.rawResults = rawResults.toArray(byte[][]::new);
        inputs.captureTimestampsUs =
                captureTimestampsUs.stream().mapToLong(Long::longValue).toArray();
        inputs.publishTimestampsUs =
                publishTimestampsUs.stream().mapToLong(Long::longValue).toArray();
    }

    /**
     * Decodes the raw PhotonVision bytes stored in {@code inputs} into {@link CameraResult}
     * records.
     *
     * <p>Each raw byte array contains a camera-index header followed by a PhotonVision struct.
     * The PhotonVision result is unpacked, then each tracked target's field-to-camera pose is
     * reconstructed using the known tag field positions.
     */
    public static CameraResult[] decodeResults(
            VisionIOInputs inputs, AprilTagFieldLayout tagLayout) {
        ArrayList<CameraResult> results = new ArrayList<>(inputs.rawResults.length);
        for (int i = 0; i < inputs.rawResults.length; i++) {
            byte[] raw = inputs.rawResults[i];
            if (raw == null || raw.length == 0) continue;

            DecodedPhotonPacket decoded = unpackPhotonPacket(raw);
            if (decoded == null) continue;

            long captureTs =
                    i < inputs.captureTimestampsUs.length ? inputs.captureTimestampsUs[i] : 0;
            long publishTs =
                    i < inputs.publishTimestampsUs.length ? inputs.publishTimestampsUs[i] : 0;

            results.add(
                    toCameraResult(
                            decoded.result(),
                            captureTs,
                            publishTs,
                            tagLayout,
                            decoded.cameraIndex()));
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

    private static DecodedPhotonPacket unpackPhotonPacket(byte[] raw) {
        if (raw.length <= PACKET_HEADER_BYTES) return null;
        ByteBuffer header = ByteBuffer.wrap(raw);
        int cameraIndex = header.getInt();
        byte[] photonPayload = Arrays.copyOfRange(raw, PACKET_HEADER_BYTES, raw.length);
        PhotonPipelineResult result = unpackPhotonResult(photonPayload);
        return result == null ? null : new DecodedPhotonPacket(result, cameraIndex);
    }

    /**
     * Converts a deserialized {@link PhotonPipelineResult} to a standardized {@link CameraResult}.
     *
     * <p>For each tracked target, the field-to-camera pose is reconstructed as: {@code
     * fieldToCamera = fieldToTag * inverse(cameraToTag)}.
     */
    static CameraResult toCameraResult(
            PhotonPipelineResult photon,
            long captureTimestampUs,
            long publishTimestampUs,
            AprilTagFieldLayout tagLayout,
            int cameraIndex) {
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
                (double) publishTimestampUs,
                cameraIndex);
    }

    static byte[] packPhotonResult(PhotonPipelineResult result, int cameraIndex) {
        Packet packet = new Packet(512);
        PhotonPipelineResult.photonStruct.pack(packet, result);
        byte[] payload = packet.getWrittenDataCopy();
        ByteBuffer encoded = ByteBuffer.allocate(PACKET_HEADER_BYTES + payload.length);
        encoded.putInt(cameraIndex);
        encoded.put(payload);
        return encoded.array();
    }
}
