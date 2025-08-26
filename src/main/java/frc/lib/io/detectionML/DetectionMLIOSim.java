// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.io.detectionML;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public class DetectionMLIOSim extends DetectionMLIOPhotonVision {
    protected final String cameraName;
    private final VisionSystemSim visionSim;
    private final PhotonCamera cam;
    private final PhotonCameraSim camSim;
    private final Supplier<Pose2d> robotPoseSupplier;

    public DetectionMLIOSim(String cameraName, Supplier<Pose2d> robotPoseSupplier)
    {
        super(cameraName);
        this.cameraName = cameraName;
        // Initialize vision sim
        cam = new PhotonCamera(cameraName);
        camSim = new PhotonCameraSim(cam, new SimCameraProperties());
        visionSim = new VisionSystemSim("objectML");
        visionSim.addCamera(camSim, new Transform3d(0, 0, 1, new Rotation3d()));
        this.robotPoseSupplier = robotPoseSupplier;

        visionSim.addVisionTargets("ALGAE",
            new VisionTargetSim(new Pose3d(3, 3, 0.5, new Rotation3d()), new TargetModel(1)));

        for (VisionTargetSim target : visionSim.getVisionTargets()) {
            Logger.recordOutput("ALGAE POSE", target.getPose());
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
