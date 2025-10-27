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
    private final Supplier<VisionTargetSim> visionTargetSupplier;
    private final VisionTargetSim[] visionTargets;
    private Set<VisionTargetSim> targetSet;
    private List<VisionTargetSim> targetList;
    private final String target_name;

    public ObjectDetectionIOSim(String cameraName, Transform3d cameraTransform,
        Supplier<Pose2d> robotPoseSupplier,
        String target_name, VisionTargetSim[] visionTargets,
        Supplier<VisionTargetSim> visionTargetSupplier)
    {
        super(cameraName);
        this.cameraName = cameraName;
        this.target_name = target_name;
        // Initialize vision sim
        cam = new PhotonCamera(cameraName);
        camSim = new PhotonCameraSim(cam, new SimCameraProperties());
        // Wireframe visualizer
        camSim.enableDrawWireframe(true);
        // Create a vision system sim and add the sim camera to it
        visionSim = new VisionSystemSim("objectDetection");
        visionSim.addCamera(camSim, cameraTransform);
        // Buffer of vision targets
        this.visionTargets = visionTargets;
        // Suppliers for dynamic update in sim
        this.robotPoseSupplier = robotPoseSupplier;
        this.visionTargetSupplier = visionTargetSupplier;
        // Add vision targets to the sim
        visionTargets[3] = visionTargetSupplier.get();
        visionSim.addVisionTargets(target_name, visionTargets);
        // Retrieve the vision targets on the sim field in a set and then convert it to a list for
        // easy indexing
        targetSet = visionSim.getVisionTargets();
        targetList = new ArrayList<>(targetSet);
        // Initialize sim target pose logging -- update in periodic below for AScope
        for (VisionTargetSim target : targetList) {
            Logger.recordOutput("TARGET POSE" + targetList.indexOf(target), target.getPose());
        }
    }

    // Update the robot's pose in the sim and use the super's implementation to update inputs
    @Override
    public void updateInputs(ObjectDetectionIOInputs inputs)
    {
        // Update robot & target poses
        visionSim.update(robotPoseSupplier.get());
        visionSim.clearVisionTargets();
        visionTargets[3] = visionTargetSupplier.get();
        visionSim.addVisionTargets(target_name, visionTargets);
        // Log updated target poses for AScope
        targetSet = visionSim.getVisionTargets();
        targetList = new ArrayList<>(targetSet);
        for (VisionTargetSim target : targetList) {
            Logger.recordOutput("TARGET POSE" + targetList.indexOf(target), target.getPose());
        }
        super.updateInputs(inputs);
    }

    @Override
    public String getCamera()
    {
        return cameraName;
    }
}
