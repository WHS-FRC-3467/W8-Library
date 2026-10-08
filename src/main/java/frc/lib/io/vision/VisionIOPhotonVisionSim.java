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

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;

import frc.lib.devices.AprilTagCamera.CameraProperties;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Simulates all configured PhotonVision cameras and logs their native PhotonVision result packets.
 *
 * <p>All simulated cameras are owned by one IO instance. Each raw packet includes its camera
 * index so replay retains per-camera attribution.
 */
public class VisionIOPhotonVisionSim implements VisionIO {
    private record SimCamera(CameraProperties properties, PhotonCamera camera) {}

    private final Supplier<Pose2d> poseSupplier;
    private final VisionSystemSim system;
    private final List<SimCamera> cameras = new ArrayList<>();

    public VisionIOPhotonVisionSim(
            CameraProperties[] cameraProperties,
            VisionSystemSim system,
            Supplier<Pose2d> poseSupplier,
            AprilTagFieldLayout fieldLayout) {
        if (cameraProperties == null || cameraProperties.length == 0) {
            throw new IllegalArgumentException("At least one simulated camera is required");
        }
        this.poseSupplier = poseSupplier;
        this.system = system;
        for (CameraProperties properties : cameraProperties) {
            PhotonCamera photonCamera = new PhotonCamera(properties.name());
            var simCameraProperties = new SimCameraProperties();
            if (properties.cameraMatrix() == null || properties.distCoeffs() == null) {
                simCameraProperties.setCalibration(
                        properties.resolutionWidth(),
                        properties.resolutionHeight(),
                        Rotation2d.fromRadians(properties.fov().in(Radians)));
            } else {
                simCameraProperties.setCalibration(
                        properties.resolutionWidth(),
                        properties.resolutionHeight(),
                        properties.cameraMatrix(),
                        properties.distCoeffs());
            }

            simCameraProperties.setFPS(properties.fps());
            simCameraProperties.setAvgLatencyMs(properties.latency().in(Units.Milliseconds));
            simCameraProperties.setLatencyStdDevMs(
                    properties.latencyStdDev().in(Units.Milliseconds));

            PhotonCameraSim cameraSim =
                    new PhotonCameraSim(photonCamera, simCameraProperties, fieldLayout);
            system.addCamera(cameraSim, properties.robotToCamera());
            cameras.add(new SimCamera(properties, photonCamera));
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        system.update(poseSupplier.get());
        inputs.connected = true;
        inputs.rawPacketType = NativePacketType.PHOTON;

        ArrayList<byte[]> rawResults = new ArrayList<>();
        ArrayList<Long> captureTimestampsUs = new ArrayList<>();
        ArrayList<Long> publishTimestampsUs = new ArrayList<>();
        for (SimCamera camera : cameras) {
            for (PhotonPipelineResult result : camera.camera().getAllUnreadResults()) {
                rawResults.add(
                        VisionIOPhotonVision.packPhotonResult(
                                result, camera.properties().index()));
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
}
