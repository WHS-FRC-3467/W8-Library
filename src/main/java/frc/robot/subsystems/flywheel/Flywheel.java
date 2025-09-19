// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Amps;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.util.LoggerHelper;

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
        LoggerHelper.recordCurrentCommand(FlywheelConstants.NAME, this);
        io.periodic();
    }

    public Command shoot()
    {
        return this.runOnce(() -> io.runVelocity(FlywheelConstants.MAX_VELOCITY,
            FlywheelConstants.MAX_ACCELERATION, PIDSlot.SLOT_0)).withName("Shoot");
    }

    public Command stop()
    {
        return this.runOnce(() -> io.runCoast()).withName("Stop");
    }

    // For unit testing
    protected Command shootAmps()
    {
        return this.runOnce(() -> io.runCurrent(Amps.of(30))).withName("Shoot Amps");
    }

    public Current getTorqueCurrent()
    {
        return io.getTorqueCurrent();
    }

    public AngularVelocity getVelocity()
    {
        return io.getVelocity();
    }

    public void close()
    {
        io.close();
    }
}
