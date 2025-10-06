// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.linear;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.linear.LinearMechanism;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggerHelper;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/** Add your docs here. */
public class Linear extends SubsystemBase {
    private final LinearMechanism io;
    private Trigger homedTrigger;
    private Debouncer homeDebouncer = new Debouncer(0.1, DebounceType.kRising);

    private static final LoggedTunableNumber STOW_SETPOINT =
        new LoggedTunableNumber("Stow Height", 0.0);
    private static final LoggedTunableNumber RASIED_SETPOINT =
        new LoggedTunableNumber("Raised Height", 30.0);

    @RequiredArgsConstructor
    @SuppressWarnings("Immutable")
    @Getter
    public enum Setpoint {
        HOME(Inches.of(0.0)),
        STOW(Inches.of(STOW_SETPOINT.get())),
        RAISED(Inches.of(RASIED_SETPOINT.get()));

        private final Distance setpoint;

        public Angle getAngle()
        {
            return LinearConstants.CONVERTER.toAngle(setpoint);
        }
    }

    public Linear(LinearMechanism io)
    {
        this.io = io;
        homedTrigger =
            new Trigger(() -> homeDebouncer.calculate(io.getSupplyCurrent().gte(Amps.of(10))));
    }

    @Override
    public void periodic()
    {
        LoggerHelper.recordCurrentCommand(LinearConstants.NAME, this);
        io.periodic();
    }

    public Command setGoal(Setpoint setpoint)
    {
        return this
            .runOnce(() -> io.runPosition(setpoint.getAngle(), LinearConstants.CRUISE_VELOCITY,
                LinearConstants.ACCELERATION, LinearConstants.JERK, PIDSlot.SLOT_0))
            .withName("Go To " + setpoint.toString() + " Setpoint");
    }

    public boolean nearGoal(Distance goalPosition)
    {
        return io.nearGoal(goalPosition, LinearConstants.TOLERANCE);
    }

    public Command waitUntilGoalCommand(Distance position)
    {
        return Commands.waitUntil(() -> {
            return nearGoal(position);
        });
    }

    public Command setGoalCommandWithWait(Setpoint setpoint)
    {
        return waitUntilGoalCommand(setpoint.getSetpoint())
            .deadlineFor(setGoal(setpoint))
            .withName("Go To " + setpoint.toString() + " Setpoint with wait");
    }

    public Command homeCommand()
    {
        return Commands.sequence(
            runOnce(() -> io.runVoltage(Volts.of(-2))),
            Commands.waitUntil(homedTrigger),
            runOnce(() -> io.setEncoderPosition(Setpoint.HOME.getAngle())),
            setGoal(Setpoint.STOW))
            .withName("Homing");
    }

    public AngularVelocity getVelocity()
    {
        return io.getVelocity();
    }

    public LinearVelocity getLinearVelocity()
    {
        return LinearConstants.CONVERTER.toDistance(io.getVelocity().times(Seconds.of(1)))
            .div(Seconds.of(1));
    }

    public void close()
    {
        io.close();
    }
}
