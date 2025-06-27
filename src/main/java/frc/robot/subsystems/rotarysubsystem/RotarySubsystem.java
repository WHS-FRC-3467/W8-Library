// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rotarysubsystem;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO;
import frc.lib.subsystems.Motor;
import frc.lib.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class RotarySubsystem extends SubsystemBase {

    private final Motor motor;

    private static final LoggedTunableNumber STOW_SETPOINT = new LoggedTunableNumber("TEST", 0.0);

    @RequiredArgsConstructor
    @Getter
    public enum State {
        STOW(Degrees.of(STOW_SETPOINT.get()));

        private final Angle setpoint;
    }


    public RotarySubsystem(MotorIO motorIO)
    {
        motor = new Motor(motorIO);

    }

    @Override
    public void periodic()
    {
        motor.periodic();
    }

    public boolean nearPosition(Angle targetPosition)
    {
        return MathUtil.isNear(
            motor.getPosition().in(BaseUnits.AngleUnit),
            targetPosition.in(BaseUnits.AngleUnit),
            RotarySubsystemConstants.TOLERANCE.in(BaseUnits.AngleUnit));
    }

    public Command waitForPositionCommand(Angle mechanismPosition)
    {
        return Commands.waitUntil(() -> {
            return nearPosition(mechanismPosition);
        });
    }

    public Command setState(State state)
    {
        return run(() -> motor.runPosition(state.setpoint, RotarySubsystemConstants.CRUISE_VELOCITY,
            RotarySubsystemConstants.ACCELERATION, RotarySubsystemConstants.JERK,
            MotorIO.PIDSlot.SLOT_1));
    };

    public Command setpointCommandWithWait(State state)
    {
        return waitForPositionCommand(state.setpoint)
            .deadlineFor(setState(state));
    }


}
