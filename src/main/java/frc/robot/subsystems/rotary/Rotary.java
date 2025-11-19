// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rotary;

import static edu.wpi.first.units.Units.Degrees;
import java.util.function.DoubleSupplier;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggerHelper;
import frc.robot.RobotState;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class Rotary extends SubsystemBase {

    private final RotaryMechanism io;

    private static final LoggedTunableNumber STOW_SETPOINT = new LoggedTunableNumber("TEST", 0.0);
    private static final LoggedTunableNumber RAISED_SETPOINT =
        new LoggedTunableNumber("RAISED", -90);

    @RequiredArgsConstructor
    @SuppressWarnings("Immutable")
    @Getter
    public enum Setpoint {
        STOW(Degrees.of(STOW_SETPOINT.get())),
        RAISED(Degrees.of(RAISED_SETPOINT.get()));

        private final Angle setpoint;
    }

    private final RobotState robotstate;


    public Rotary(RotaryMechanism io)
    {
        this.io = io;
        this.robotstate = RobotState.getInstance();
        setSetpoint(RotaryConstants.DEFAULT_SETPOINT).ignoringDisable(true).schedule();
    }

    @Override
    public void periodic()
    {
        LoggerHelper.recordCurrentCommand(RotaryConstants.NAME, this);
        io.periodic();
        robotstate.setRotaryPose(io.getPoseSupplier().get());

    }

    public Command setSetpoint(Setpoint setpoint)
    {
        return this.runOnce(
            () -> io.runPosition(setpoint.getSetpoint(), RotaryConstants.CRUISE_VELOCITY,
                RotaryConstants.ACCELERATION, RotaryConstants.JERK,
                PIDSlot.SLOT_0))
            .withName("Go To " + setpoint.toString() + " Setpoint");
    };

    public void setSetpoint(Angle angle)
    {
        io.runPosition(angle, RotaryConstants.CRUISE_VELOCITY,
                RotaryConstants.ACCELERATION, RotaryConstants.JERK,
                PIDSlot.SLOT_0);   
    }

    public boolean nearGoal(Angle targetPosition)
    {
        return io.nearGoal(targetPosition, RotaryConstants.TOLERANCE);
    }

    public Command waitUntilGoalCommand(Angle position)
    {
        return Commands.waitUntil(() -> {
            return nearGoal(position);
        });
    }

    public Command setGoalCommandWithWait(Setpoint setpoint)
    {
        return waitUntilGoalCommand(setpoint.getSetpoint())
            .deadlineFor(setSetpoint(setpoint))
            .withName("Go To " + setpoint.toString() + " Setpoint with wait");
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
