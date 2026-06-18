// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.lib.autos.Auto;
import frc.robot.RobotState;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.drive.Drive;

import java.util.List;

/** Add your docs here. */
public class ExampleAuto {

    public static Auto exampleAuto(Drive drive) {
        var p1 = new Pose2d(4.45, 7.4, Rotation2d.fromDegrees(90.0));
        var p2 = new Pose2d(7.5, 6.86, Rotation2d.fromRadians(1.17));
        var p3 = new Pose2d(7.12, 4.65, Rotation2d.fromDegrees(0.0));
        var p4 = new Pose2d(6.18, 5.41, Rotation2d.fromRadians(4.71));
        var p5 = new Pose2d(3.89, 5.41, Rotation2d.fromRadians(4.71));
        var p6 = new Pose2d(3.10, 5.2, Rotation2d.fromRadians(-0.68));

        return new Auto(
                "Example Auto",
                Commands.sequence(
                        Commands.runOnce(() -> RobotState.getInstance().resetPose(p1)),
                        // new DriveToPose(drive, () -> p1).withDistanceTolerance(Inches.of(16.0)),
                        new DriveToPose(drive, () -> p2).withDistanceTolerance(Inches.of(6.0)),
                        new DriveToPose(drive, () -> p3).withDistanceTolerance(Inches.of(14.0)),
                        new DriveToPose(drive, () -> p4).withDistanceTolerance(Inches.of(14.0)),
                        new DriveToPose(drive, () -> p5).withDistanceTolerance(Inches.of(14.0)),
                        new DriveToPose(drive, () -> p6).withDistanceTolerance(Inches.of(14.0))),
                List.of(p1, p2, p3, p4, p5, p6));
    }
}
