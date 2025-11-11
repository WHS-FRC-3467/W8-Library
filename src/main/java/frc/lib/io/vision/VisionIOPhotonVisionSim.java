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
import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * Simulated implementation of {@link VisionIOPhotonVision} using the PhotonVision simulation
 * framework.
 *
 * <p>
 * This class connects a {@link PhotonCameraSim} to a {@link VisionSystemSim} to simulate the
 * behavior of a real PhotonVision camera in a physics-based environment. It allows the robot code
 * to receive realistic vision data based on the robot's simulated pose and the field's AprilTag
 * layout.
 */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
    private final Supplier<Pose2d> poseSupplier;
    private final PhotonCameraSim cameraSim;
    private final VisionSystemSim system;

    /**
     * Constructs a new simulated PhotonVision IO implementation.
     *
     * @param camera The virtual {@link CameraProperties} configuration.
     * @param system The {@link VisionSystemSim} managing simulated vision sources.
     * @param poseSupplier A {@link Supplier} that provides the robot's current {@link Pose2d} in
     *        simulation.
     * @param fieldLayout The {@link AprilTagFieldLayout} describing all AprilTags in the field.
     */
    public VisionIOPhotonVisionSim(
        CameraProperties camera,
        VisionSystemSim system,
        Supplier<Pose2d> poseSupplier,
        AprilTagFieldLayout fieldLayout) {
        super(camera);
        this.poseSupplier = poseSupplier;
        this.system = system;

        var simCameraProperties = new SimCameraProperties();
        simCameraProperties.setCalibration(
            camera.resolutionWidth(),
            camera.resolutionHeight(),
            camera.cameraMatrix(),
            camera.distCoeffs());

        cameraSim = new PhotonCameraSim(super.photonCamera, simCameraProperties, fieldLayout);
        this.system.addCamera(cameraSim, camera.robotToCamera());
    }

    /**
     * Updates the input data from the simulated PhotonVision system.
     *
     * <p>
     * This method updates the vision simulation with the current robot pose and calls the
     * superclass to populate the {@link VisionIOInputs} with simulated camera observations and pose
     * estimates.
     *
     * @param inputs The structure to store the latest simulated vision data.
     */
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        system.update(poseSupplier.get());
        super.updateInputs(inputs);
    }
}
