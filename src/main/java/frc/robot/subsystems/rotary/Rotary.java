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
    private static LoggedTunableNumber STOW_SETPOINT =
        new LoggedTunableNumber("Rotary STOW", 0.0);
    private static LoggedTunableNumber RAISED_SETPOINT =
        new LoggedTunableNumber("Rotary RAISED", -90);

    @RequiredArgsConstructor
    @Getter
    public enum Setpoint {
        HOME(null),
        STOW(STOW_SETPOINT),
        RAISED(RAISED_SETPOINT);

        private final LoggedTunableNumber tunableNumber;

        public Angle getSetpoint()
        {
            if (tunableNumber == null) {
                return Degrees.of(0.0);
            }
            return Degrees.of(tunableNumber.get());
        }
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
