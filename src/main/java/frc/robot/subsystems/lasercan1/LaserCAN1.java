// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lasercan1;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystems.DistanceSensorSubsystem;

/** Add your docs here. */
public class LaserCAN1 extends DistanceSensorSubsystem {

    public LaserCAN1()
    {
        super(LaserCAN1Constants.NAME, LaserCAN1Constants.getDistanceSensorIO());
    }

    public Command waitUntilBetweenDistance(Distance min, Distance max)
    {
        return Commands.waitUntil(() -> (getDistance().gt(min) && getDistance().lt(max)));
    }
}
