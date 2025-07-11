// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;

/** Add your docs here. */
public class Flywheel extends SubsystemBase { // Don't extend if contained in superstructure
    private final FlywheelMechanism io;

    public Flywheel(FlywheelMechanism io)
    {
        this.io = io;
    }

    @Override
    public void periodic()
    {
        io.periodic();
    }
}
