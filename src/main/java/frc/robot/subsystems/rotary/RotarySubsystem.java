// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rotary;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import frc.lib.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class RotarySubsystem extends SubsystemBase {

    private final RotaryMechanism io;

    private static final LoggedTunableNumber STOW_SETPOINT = new LoggedTunableNumber("TEST", 0.0);

    @RequiredArgsConstructor
    @Getter
    public enum State {
        STOW(Degrees.of(STOW_SETPOINT.get()));

        private final Angle setpoint;
    }


    public RotarySubsystem(RotaryMechanism io)
    {
        this.io = io;

    }

    @Override
    public void periodic()
    {
        io.periodic();
    }

    // public boolean nearPosition(Angle targetPosition)
    // {
    // return MathUtil.isNear(
    // io.getPosition().in(BaseUnits.AngleUnit),
    // targetPosition.in(BaseUnits.AngleUnit),
    // RotarySubsystemConstants.TOLERANCE.in(BaseUnits.AngleUnit));
    // }

    // public Command waitForPositionCommand(Angle mechanismPosition)
    // {
    // return Commands.waitUntil(() -> {
    // return nearPosition(mechanismPosition);
    // });
    // }

    public Command setState(State state)
    {
        return run(() -> io.runPosition(state.setpoint, RotarySubsystemConstants.CRUISE_VELOCITY,
            RotarySubsystemConstants.ACCELERATION, RotarySubsystemConstants.JERK,
            PIDSlot.SLOT_1));
    };

    // public Command setpointCommandWithWait(State state)
    // {
    // return waitForPositionCommand(state.setpoint)
    // .deadlineFor(setState(state));
    // }


}
