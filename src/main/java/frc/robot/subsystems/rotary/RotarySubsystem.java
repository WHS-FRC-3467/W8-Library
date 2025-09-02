// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rotary;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggerHelper;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class RotarySubsystem extends SubsystemBase {

    private final RotaryMechanism io;

    private static final LoggedTunableNumber STOW_SETPOINT = new LoggedTunableNumber("TEST", 0.0);
    private static final LoggedTunableNumber RAISED_SETPOINT =
        new LoggedTunableNumber("RAISED", 90);

    @RequiredArgsConstructor
    @Getter
    public enum Setpoint {
        STOW(Degrees.of(STOW_SETPOINT.get())),
        RAISED(Degrees.of(RAISED_SETPOINT.get()));

        private final Angle setpoint;
    }


    public RotarySubsystem(RotaryMechanism io)
    {
        this.io = io;

    }

    @Override
    public void periodic()
    {
        LoggerHelper.recordCurrentCommand(this);
        io.periodic();
    }

    public Command setSetpoint(Setpoint setpoint)
    {
        return this.runOnce(
            () -> io.runPosition(setpoint.getSetpoint(), RotarySubsystemConstants.CRUISE_VELOCITY,
                RotarySubsystemConstants.ACCELERATION, RotarySubsystemConstants.JERK,
                PIDSlot.SLOT_1))
                .withName("Go To " + setpoint.toString() + " Setpoint");
    };

    public boolean nearPosition(Angle targetPosition)
    {
        return MathUtil.isNear(
            io.getPosition().in(BaseUnits.AngleUnit),
            targetPosition.in(BaseUnits.AngleUnit),
            RotarySubsystemConstants.TOLERANCE.in(BaseUnits.AngleUnit));
    }

    public Command waitForPositionCommand(Angle position)
    {
        return Commands.waitUntil(() -> {
            return nearPosition(position);
        }).withName("Wait for position " + position.toString());
    }

    public Command setpointCommandWithWait(Setpoint setpoint)
    {
        return waitForPositionCommand(setpoint.getSetpoint())
            .deadlineFor(setSetpoint(setpoint))
            .withName("Go To " + setpoint.toString() + " Setpoint with wait");
    }

    public AngularVelocity getVelocity() {
        return io.getVelocity();
    }
}
