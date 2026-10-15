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

import frc.lib.devices.VisionSource.CameraProperties;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import java.util.List;
import java.util.function.Supplier;

/**
 * {@link VisionIOPhotonVision} with PhotonVision's simulator: each camera is simulated from the
 * robot's pose and the field's tag layout.
 */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
    private final Supplier<Pose2d> poseSupplier;
    private final VisionSystemSim system;

    /**
     * @param cameras the cameras of this source
     * @param poseSupplier the robot pose to simulate the cameras from
     * @param fieldLayout the field's tags
     */
    public VisionIOPhotonVisionSim(
            List<CameraProperties> cameras,
            Supplier<Pose2d> poseSupplier,
            AprilTagFieldLayout fieldLayout) {
        super(cameras);
        this.poseSupplier = poseSupplier;
        this.system = new VisionSystemSim("main");
        system.addAprilTags(fieldLayout);
        for (int i = 0; i < cameras.size(); i++) {
            CameraProperties camera = cameras.get(i);
            PhotonCameraSim cameraSim =
                    new PhotonCameraSim(photonCameras.get(i), simProperties(camera), fieldLayout);
            system.addCamera(cameraSim, camera.robotToCamera());
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        system.update(poseSupplier.get()); // once for all cameras
        super.updateInputs(inputs);
    }

    private static SimCameraProperties simProperties(CameraProperties camera) {
        var properties = new SimCameraProperties();
        if (camera.cameraMatrix() == null || camera.distCoeffs() == null) {
            properties.setCalibration(
                    camera.resolutionWidth(),
                    camera.resolutionHeight(),
                    Rotation2d.fromRadians(camera.fov().in(Radians)));
        } else {
            properties.setCalibration(
                    camera.resolutionWidth(),
                    camera.resolutionHeight(),
                    camera.cameraMatrix(),
                    camera.distCoeffs());
        }
        properties.setFPS(camera.fps());
        properties.setAvgLatencyMs(camera.latency().in(Units.Milliseconds));
        properties.setLatencyStdDevMs(camera.latencyStdDev().in(Units.Milliseconds));
        return properties;
    }
}
