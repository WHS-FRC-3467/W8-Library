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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
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
    public VisionIOPhotonVisionSim(
        String name,
        Transform3d robotToCamera,
        VisionSystemSim system,
        Supplier<Pose2d> poseSupplier,
        AprilTagFieldLayout fieldLayout)
    {
        super(name, robotToCamera);
        this.poseSupplier = poseSupplier;

        this.system = system;

        // Add sim camera
        var cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(1600, 1304,
            MatBuilder.fill(Nat.N3(), Nat.N3(),
                2002.948392331919, 0.0, 783.9099067246102,
                0.0, 1999.0390684862123, 662.7694019679813,
                0.0, 0.0, 1.0),
            VecBuilder.fill(
                0.09905119793103302,
                -0.06388083628565337,
                3.87402720846368E-5,
                1.4421218015997156E-4,
                -0.16329892957216433,
                -0.004599206903333014,
                0.0029050841273878885,
                0.0067195798658376375));
        cameraProperties.setFPS(60);
        cameraSim = new PhotonCameraSim(camera, cameraProperties, fieldLayout);
        this.system.addCamera(cameraSim, robotToCamera);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs)
    {
        system.update(poseSupplier.get());
        super.updateInputs(inputs);
    }
}
