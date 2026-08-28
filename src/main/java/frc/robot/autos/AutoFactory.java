// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import static edu.wpi.first.units.Units.Degrees;
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
public class AutoFactory {
    private Drive drive;

    public AutoFactory(Drive drive) {
        this.drive = drive;
    }

    public Auto noneAuto() {
        return new Auto("None Auto", Commands.none(), List.of(new Pose2d()));
    }

    public Auto leftNeutralSweep() {
        List<Pose2d> points =
                List.of(
                        new Pose2d(4.45, 7.4, Rotation2d.fromDegrees(90.0)), // Starting pose
                        new Pose2d(5.65, 7.4, Rotation2d.fromDegrees(90.0)), // Tunnel Exit
                        new Pose2d(8.26, 6.83, Rotation2d.fromDegrees(90.0)), // Midline Entry
                        new Pose2d(8.22, 4.653, Rotation2d.fromRadians(1.173)), // Midline Exit
                        new Pose2d(7.364, 4.254, Rotation2d.fromRadians(-1.22)), // Midfield Hook
                        new Pose2d(6.18, 5.41, Rotation2d.fromRadians(4.71)), // Bump Entry
                        new Pose2d(3.89, 5.41, Rotation2d.fromRadians(4.71)), // Bump Exit
                        new Pose2d(3.10, 5.2, Rotation2d.fromRadians(-0.68)) // Shoot Pose
                        );

        return new Auto(
                "Example Auto",
                Commands.sequence(
                        Commands.runOnce(() -> RobotState.getInstance().resetPose(points.get(0))),
                        Commands.waitSeconds(1.0),
                        new DriveToPose(drive, () -> points.get(1))
                                .withDistanceTolerance(Inches.of(6.0)),
                        new DriveToPose(drive, () -> points.get(2))
                                .withDistanceTolerance(Inches.of(12.0)),
                        new DriveToPose(drive, () -> points.get(3))
                                .withDistanceTolerance(Inches.of(12.0)),
                        new DriveToPose(drive, () -> points.get(4))
                                .withDistanceTolerance(Inches.of(12.0)),
                        new DriveToPose(drive, () -> points.get(5))
                                .withDistanceTolerance(Inches.of(10.0))
                                .withAngularTolerance(Degrees.of(5.0)),
                        new DriveToPose(drive, () -> points.get(6))
                                .withDistanceTolerance(Inches.of(10.0)),
                        new DriveToPose(drive, () -> points.get(7))
                                .withDistanceTolerance(Inches.of(6.0))),
                points);
    }
}
