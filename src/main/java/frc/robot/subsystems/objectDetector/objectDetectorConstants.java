// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.objectDetector;

import frc.lib.io.detectionML.*;

/** Add your docs here. */
public class objectDetectorConstants {
    public final static String NAME = "Detection Camera #1";

    public static DetectionMLIOPhotonVision getReal()
    {
        return new DetectionMLIOPhotonVision(NAME);
    }

    public static DetectionMLIOSim getSim()
    {
        return new DetectionMLIOSim(NAME);
    }

    public static DetectionMLIO getReplay()
    {
        return new DetectionMLIO() {};
    }
}
