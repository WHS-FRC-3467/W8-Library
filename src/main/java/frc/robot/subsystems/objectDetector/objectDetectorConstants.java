// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.objectDetector;

import java.util.function.Supplier;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionTargetSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.io.detectionML.*;

/** Add your docs here. */
public class objectDetectorConstants {
    public final static String CAMERA0_NAME = "Detection Camera #0";
    public final static double cameraRoll = 0.0; // degrees
    public final static double cameraPitch = -10.0; // degrees
    public final static double cameraYaw = 0.0; // degrees
    public final static double cameraZ = 1.00; // meters
    public static Transform3d CAMERA0TRANSFORM =
        new Transform3d(0.0, 0.0, cameraZ,
            new Rotation3d(Math.toRadians(cameraRoll), Math.toRadians(cameraPitch),
                Math.toRadians(cameraYaw)));

    public final static String ALGAE_NAME = "Algae";
    public final static double algaeHeightMeters = 0.41;

    public final static VisionTargetSim[] ALGAE_TARGETS = {
            new VisionTargetSim(new Pose3d(3, 3, algaeHeightMeters / 2, new Rotation3d()),
                new TargetModel(algaeHeightMeters))
            // ,
            // new VisionTargetSim(new Pose3d(5, 5, algaeHeightMeters / 2, new Rotation3d()),
            // new TargetModel(algaeHeightMeters))
    };

    public static DetectionMLIOPhotonVision getReal()
    {
        return new DetectionMLIOPhotonVision(CAMERA0_NAME);
    }

    public static DetectionMLIOSim getSim(Supplier<Pose2d> robotPoseSupplier)
    {
        return new DetectionMLIOSim(CAMERA0_NAME, CAMERA0TRANSFORM, robotPoseSupplier, ALGAE_NAME,
            ALGAE_TARGETS);
    }

    public static DetectionMLIO getReplay()
    {
        return new DetectionMLIO() {};
    }
}
