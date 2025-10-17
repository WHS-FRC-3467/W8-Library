// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.io.objectDetection;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

/** An object detection sim class that utilizes the PhotonVision implementation for tests. */
public class ObjectDetectionIOSim extends ObjectDetectionIOPhotonVision {
    protected final String cameraName;
    private final VisionSystemSim visionSim;
    private final PhotonCamera cam;
    private final PhotonCameraSim camSim;
    private final Supplier<Pose2d> robotPoseSupplier;

    public ObjectDetectionIOSim(String cameraName, Transform3d cameraTransform,
        Supplier<Pose2d> robotPoseSupplier,
        String target_name, VisionTargetSim[] targets)
    {
        super(cameraName);
        this.cameraName = cameraName;
        // Initialize vision sim
        cam = new PhotonCamera(cameraName);
        camSim = new PhotonCameraSim(cam, new SimCameraProperties());
        // Wireframe visualizer
        camSim.enableDrawWireframe(true);

        visionSim = new VisionSystemSim("objectDetection");
        visionSim.addCamera(camSim, cameraTransform);
        this.robotPoseSupplier = robotPoseSupplier;

        // Add vision targets to the sim
        visionSim.addVisionTargets(target_name, targets);
        // Retrieve the vision targets on the sim field in a set and then convert it to a list for
        // easy indexing
        Set<VisionTargetSim> targetSet = visionSim.getVisionTargets();
        List<VisionTargetSim> targetList = new ArrayList<>(targetSet);
        // Log the poses of each target for debugging purposes
        for (VisionTargetSim target : targetList) {
            Logger.recordOutput("ALGAE POSE" + targetList.indexOf(target), target.getPose());
        }

    }

    // Update the robot's pose in the sim and use the super's implementation to update inputs
    @Override
    public void updateInputs(ObjectDetectionIOInputs inputs)
    {
        visionSim.update(robotPoseSupplier.get());
        super.updateInputs(inputs);
    }

    @Override
    public String getCamera()
    {
        return cameraName;
    }

}
