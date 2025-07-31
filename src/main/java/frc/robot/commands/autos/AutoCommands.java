// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class AutoCommands {

    public static Command resetOdom(Drive drive, PathPlannerPath path)
    {
        return drive.runOnce(
            () -> {
                Pose2d pose =
                    path.getStartingHolonomicPose().get();
                // if (isRedAlliance()) {
                // pose = GeometryUtil.flipFieldPose(pose);
                // }

                drive.setPose(pose);
            });
    }

    // public static boolean isRedAlliance()
    // {
    // return DriverStation.getAlliance()
    // .filter(value -> value == DriverStation.Alliance.Red)
    // .isPresent();
    // }
}
