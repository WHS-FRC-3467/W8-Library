// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.lib.autos.Auto;
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
}
