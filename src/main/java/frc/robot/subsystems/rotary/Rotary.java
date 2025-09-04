/* Copyright (C) 2025 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

package frc.robot.subsystems.rotary;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import frc.lib.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class Rotary extends SubsystemBase {

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


    public Rotary(RotaryMechanism io)
    {
        this.io = io;

    }

    @Override
    public void periodic()
    {
        io.periodic();
    }

    public Command setGoal(Setpoint setpoint)
    {
        return this.runOnce(
            () -> io.runPosition(setpoint.getSetpoint(), RotaryConstants.CRUISE_VELOCITY,
                RotaryConstants.ACCELERATION, RotaryConstants.JERK,
                PIDSlot.SLOT_1));
    };

    public boolean nearGoal(Angle targetPosition)
    {
        return MathUtil.isNear(
            io.getPosition().in(BaseUnits.AngleUnit),
            targetPosition.in(BaseUnits.AngleUnit),
            RotaryConstants.TOLERANCE.in(BaseUnits.AngleUnit));
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
            .deadlineFor(setGoal(setpoint));
    }
}