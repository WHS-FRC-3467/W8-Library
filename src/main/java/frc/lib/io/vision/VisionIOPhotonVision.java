package frc.lib.io.vision;

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

import frc.lib.devices.VisionSource.CameraProperties;

import org.photonvision.PhotonCamera;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;
import java.util.List;

/**
 * {@link VisionIO} for PhotonVision cameras.
 *
 * <p>PhotonVision shows the robot only cameras (by name, on NetworkTables), not the coprocessors
 * running them, so this source is just the cameras given to it. It has no health signal of its own:
 * {@code connected} is always true and each camera's status is its own heartbeat.
 *
 * <p>Results are logged as raw PhotonVision bytes and decoded by {@link PhotonDecoder}.
 */
public class VisionIOPhotonVision implements VisionIO {
    protected final List<PhotonCamera> photonCameras;

    /**
     * @param cameras the cameras of this source, by their PhotonVision names
     */
    public VisionIOPhotonVision(List<CameraProperties> cameras) {
        this.photonCameras = cameras.stream().map(c -> new PhotonCamera(c.name())).toList();
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.rawPacketType = NativePacketType.PHOTON;
        inputs.reportsSourceHealth = false;
        inputs.connected = true;
        inputs.camerasConnected = new boolean[photonCameras.size()];

        ArrayList<byte[]> raw = new ArrayList<>();
        ArrayList<Integer> cameras = new ArrayList<>();
        ArrayList<Long> capture = new ArrayList<>();
        ArrayList<Long> publish = new ArrayList<>();
        for (int i = 0; i < photonCameras.size(); i++) {
            PhotonCamera camera = photonCameras.get(i);
            inputs.camerasConnected[i] = camera.isConnected();
            if (!inputs.camerasConnected[i]) {
                continue;
            }
            for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
                raw.add(pack(result));
                cameras.add(i);
                capture.add(result.metadata.captureTimestampMicros);
                publish.add(result.metadata.publishTimestampMicros);
            }
        }
        inputs.rawResults = raw.toArray(byte[][]::new);
        inputs.resultCameras = cameras.stream().mapToInt(Integer::intValue).toArray();
        inputs.captureTimestampsUs = capture.stream().mapToLong(Long::longValue).toArray();
        inputs.publishTimestampsUs = publish.stream().mapToLong(Long::longValue).toArray();
    }

    /** Serializes a PhotonVision result for logging; {@link PhotonDecoder} reverses it. */
    static byte[] pack(PhotonPipelineResult result) {
        Packet packet = new Packet(512);
        PhotonPipelineResult.photonStruct.pack(packet, result);
        return packet.getWrittenDataCopy();
    }
}
