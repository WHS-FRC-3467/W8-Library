// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.objectDetector;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radian;
import java.util.function.Supplier;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionTargetSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.Units;
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
    public final static Angle CAMERA0_ROLL = Units.Degrees.of(0.0);
    public final static Angle CAMERA0_PITCH = Units.Degrees.of(25.0);
    public final static Angle CAMERA0_YAW = Units.Degrees.of(0.0);
    public final static double CAMERA0_X = 0.30;
    public final static double CAMERA0_Y = -0.30;
    public final static double CAMERA0_Z = 1.0;

    public static Transform3d CAMERA0_TRANSFORM =
        new Transform3d(CAMERA0_X, CAMERA0_Y, CAMERA0_Z,
            new Rotation3d(CAMERA0_ROLL.in(Units.Radians), CAMERA0_PITCH.in(Units.Radians),
                CAMERA0_YAW.in(Units.Radians)));

    // Object detection camera # 1
    // ...

    // Real implementation of camera; call real IO layer
    public static ObjectDetectionIOPhotonVision getReal()
    {
        return new ObjectDetectionIOPhotonVision(CAMERA0_NAME);
    }

    // Simulated implementation of camera; call sim IO layer
    // 2025 Simulated Algae Targets
    public final static String SIM_NAME = "Algae";
    public final static double algaeHeightMeters = 0.41;
    // Initialize fixed array of sim targets
    public static VisionTargetSim[] SIM_TARGETS = new VisionTargetSim[] {
            new VisionTargetSim(new Pose3d(3, 2, algaeHeightMeters / 2, new Rotation3d()),
                new TargetModel(algaeHeightMeters)),
            new VisionTargetSim(new Pose3d(7, 6, algaeHeightMeters / 2, new Rotation3d()),
                new TargetModel(algaeHeightMeters)),
            new VisionTargetSim((new Pose3d(12, 7, algaeHeightMeters / 2, new Rotation3d())),
                new TargetModel(algaeHeightMeters)),
            null,
    };
    // Dynamic supplier for moving sim targets
    public static Supplier<VisionTargetSim[]> visionTargetSimSupplier =
        () -> SIM_TARGETS = new VisionTargetSim[] {
                new VisionTargetSim(new Pose3d(3, 2, algaeHeightMeters / 2, new Rotation3d()),
                    new TargetModel(algaeHeightMeters)),
                new VisionTargetSim(new Pose3d(7, 6, algaeHeightMeters / 2, new Rotation3d()),
                    new TargetModel(algaeHeightMeters)),
                new VisionTargetSim((new Pose3d(12, 7, algaeHeightMeters / 2, new Rotation3d())),
                    new TargetModel(algaeHeightMeters)),
                new VisionTargetSim(
                    (new Pose3d(16, 3.5 * Math.sin(0.25 * Math.PI * Timer.getFPGATimestamp()) + 4.1,
                        algaeHeightMeters / 2, new Rotation3d())),
                    new TargetModel(algaeHeightMeters)),
        };

    // 2026 Targets
    // ...

    // Simulate the camera(s) with the given robot pose supplier. Return an array of IOSims as
    // necessary.
    public static ObjectDetectionIOSim getSim(Supplier<Pose2d> robotPoseSupplier)
    {
        return new ObjectDetectionIOSim(CAMERA0_NAME, CAMERA0_TRANSFORM, robotPoseSupplier,
            SIM_NAME, visionTargetSimSupplier);
    }

    // Replay implementation of camera; return bare IO layer results
    public static ObjectDetectionIO getReplay()
    {
        return new ObjectDetectionIO() {};
    }
}
