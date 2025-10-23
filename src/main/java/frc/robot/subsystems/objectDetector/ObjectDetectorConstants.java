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
import edu.wpi.first.wpilibj.Timer;
import frc.lib.io.objectDetection.*;

/*
 * Subsystem constants (e.g. names, transforms) for the various object detector cameras on the
 * robot. Used to create object detector subsystems contained in RobotContainer.
 */
public class ObjectDetectorConstants {
    /*
     * Transform sign convention: +X -> towards other alliance's station, +Y -> towards center of
     * field from starting starboard edge, +theta -> right-hand rule. units: meters & degrees.
     */
    // Object detection camera # 0
    public final static String CAMERA0_NAME = "Detection Camera #0";
    public final static double CAMERA0_ROLL = 0.0;
    public final static double CAMERA0_PITCH = 25;
    public final static double CAMERA0_YAW = 0.0;
    public final static double CAMERA0_X = 0.30;
    public final static double CAMERA0_Y = -0.30;
    public final static double CAMERA0_Z = 1;

    public static Transform3d CAMERA0_TRANSFORM =
        new Transform3d(CAMERA0_X, CAMERA0_Y, CAMERA0_Z,
            new Rotation3d(Math.toRadians(CAMERA0_ROLL), Math.toRadians(CAMERA0_PITCH),
                Math.toRadians(CAMERA0_YAW)));

    // Object detection camera # 1
    // ...

    // Real implementation of camera; call real IO layer
    public static ObjectDetectionIOPhotonVision getReal()
    {
        return new ObjectDetectionIOPhotonVision(CAMERA0_NAME);
    }

    // Simulated implementation of camera; call sim IO layer
    // Simulated targets
    // 2025 Algae Targets
    public final static String SIM_NAME = "Algae";
    public final static double algaeHeightMeters = 0.41;
    public final static VisionTargetSim[] SIM_TARGETS = {
            new VisionTargetSim(new Pose3d(3, 3, algaeHeightMeters / 2, new Rotation3d()),
                new TargetModel(algaeHeightMeters)),
            new VisionTargetSim(new Pose3d(5, 6, algaeHeightMeters / 2, new Rotation3d()),
                new TargetModel(algaeHeightMeters)),
            new VisionTargetSim((new Pose3d(10, 5, algaeHeightMeters / 2, new Rotation3d())),
                new TargetModel(algaeHeightMeters)),
            new VisionTargetSim(
                (new Pose3d(12, 12 * Math.sin(0.5 * Math.PI * Timer.getFPGATimestamp()),
                    algaeHeightMeters / 2, new Rotation3d())),
                new TargetModel(algaeHeightMeters)) // 0.25 Hz oscillation
    };

    // 2026 ??? Targets
    // ...

    // Simulate the camera(s) with the given robot pose supplier. Return an array of IOSims as
    // necessary.
    public static ObjectDetectionIOSim getSim(Supplier<Pose2d> robotPoseSupplier)
    {
        return new ObjectDetectionIOSim(CAMERA0_NAME, CAMERA0_TRANSFORM, robotPoseSupplier,
            SIM_NAME,
            SIM_TARGETS);
    }

    // Replay implementation of camera; return bare IO layer results
    public static ObjectDetectionIO getReplay()
    {
        return new ObjectDetectionIO() {};
    }
}
