// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.objectDetector;

import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.io.detectionML.*;

/** Add your docs here. */
public class objectDetectorConstants {
    public final static String NAME = "Detection Camera #1";

    public static DetectionMLIOPhotonVision getReal()
    {
        return new DetectionMLIOPhotonVision(NAME);
    }

    public static DetectionMLIOSim getSim(Supplier<Pose2d> robotPoseSupplier)
    {
        return new DetectionMLIOSim(NAME, robotPoseSupplier);
    }

    public static DetectionMLIO getReplay()
    {
        return new DetectionMLIO() {};
    }
}
