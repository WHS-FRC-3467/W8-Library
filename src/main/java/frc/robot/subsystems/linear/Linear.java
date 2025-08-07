// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.linear;

import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.linear.LinearMechanism;
import frc.lib.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/** Add your docs here. */
public class Linear extends SubsystemBase {
    private final LinearMechanism io;

    private static final LoggedTunableNumber STOW_SETPOINT =
        new LoggedTunableNumber("Stow Height", 0.0);
    private static final LoggedTunableNumber RASIED_SETPOINT =
        new LoggedTunableNumber("Raised Height", 30.0);

    @RequiredArgsConstructor
    @Getter
    public enum Setpoint {
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
    }

    @Override
    public void periodic()
    {
        io.periodic();
    }

    public Command goToSetpoint(Setpoint setpoint)
    {
        return this
            .runOnce(() -> io.runPosition(setpoint.getAngle(), LinearConstants.CRUISE_VELOCITY,
                LinearConstants.ACCELERATION, LinearConstants.JERK, PIDSlot.SLOT_1));
    }
}
