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

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.util.Timestamped;
import java.util.function.Supplier;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
    private final Supplier<Pose2d> poseSupplier;
    private final PhotonCameraSim cameraSim;

    private final VisionSystemSim system;

    /**
     * Creates a new VisionIOPhotonVisionSim.
     *
     * @param name The name of the camera.
     * @param poseSupplier Supplier for the robot pose to use in simulation.
     */
    public VisionIOPhotonVisionSim(Supplier<Pose2d> poseSupplier, String name,
        Transform3d robotToCamera,
        AprilTagFieldLayout fieldLayout, PoseStrategy strategy, VisionSystemSim system)
    {
        super(name, robotToCamera, fieldLayout, strategy);
        this.poseSupplier = poseSupplier;

        this.system = system;

        // Add sim camera
        var cameraProperties = new SimCameraProperties();
        cameraSim = new PhotonCameraSim(camera, cameraProperties, aprilTagLayout);
        this.system.addCamera(cameraSim, robotToCamera);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs, Timestamped<Rotation2d> timestampedHeading)
    {
        system.update(poseSupplier.get());
        super.updateInputs(inputs, timestampedHeading);
    }
}
