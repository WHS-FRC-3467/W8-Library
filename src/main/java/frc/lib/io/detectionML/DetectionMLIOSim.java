// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.io.detectionML;

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

/** Add your docs here. */
public class DetectionMLIOSim extends DetectionMLIOPhotonVision {
    protected final String cameraName;
    private final VisionSystemSim visionSim;
    private final PhotonCamera cam;
    private final PhotonCameraSim camSim;
    private final Supplier<Pose2d> robotPoseSupplier;

    public DetectionMLIOSim(String cameraName, Transform3d cameraTransform,
        Supplier<Pose2d> robotPoseSupplier,
        String target_name, VisionTargetSim[] targets)
    {
        super(cameraName);
        this.cameraName = cameraName;
        // Initialize vision sim
        cam = new PhotonCamera(cameraName);
        camSim = new PhotonCameraSim(cam, new SimCameraProperties());
        visionSim = new VisionSystemSim("objectML");
        visionSim.addCamera(camSim, cameraTransform);
        this.robotPoseSupplier = robotPoseSupplier;

        // TODO: find cleaner impl
        visionSim.addVisionTargets(target_name, targets);
        Set<VisionTargetSim> test = visionSim.getVisionTargets();
        List<VisionTargetSim> targetList = new ArrayList<>(test);

        for (VisionTargetSim target : targetList) {
            Logger.recordOutput("ALGAE POSE" + targetList.indexOf(target), target.getPose());
        }

    }

    @Override
    public void updateInputs(DetectionMLIOInputs inputs)
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
