// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rotary;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
        HOME(Degrees.of(0.0)),
        STOW(Degrees.of(STOW_SETPOINT.get())),
        RAISED(Degrees.of(RAISED_SETPOINT.get()));

        private final Angle setpoint;
    }

    private Debouncer homeDebouncer = new Debouncer(0.1, DebounceType.kRising);
    private Trigger homedTrigger;

    private final RobotState robotstate;
    private Setpoint setpoint = Setpoint.STOW;

    public Rotary(RotaryMechanism io)
    {
        this.io = io;
        this.robotstate = RobotState.getInstance();
        setSetpoint(RotaryConstants.DEFAULT_SETPOINT)
            .ignoringDisable(true)
            .schedule();
        homedTrigger =
            new Trigger(() -> homeDebouncer.calculate(io.getSupplyCurrent().gte(Amps.of(10))));

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

    public Command setStateCommand(Setpoint setpoint)
    {
        return this.runOnce(() -> this.setpoint = setpoint)
            .withName("Elevator Set State: " + setpoint.name());
    }

    public Command homeCommand()
    {
        return Commands.sequence(runOnce(() -> io.runVoltage(Volts.of(-2))),
            Commands.waitUntil(homedTrigger),
            runOnce(() -> io.setEncoderPosition(Setpoint.HOME.getSetpoint())),
            this.setStateCommand(Setpoint.STOW))
            .withName("Homing");

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
