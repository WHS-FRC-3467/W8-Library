/*
 * Copyright (C) 2026 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */
package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.mechanisms.flywheel.FlywheelMechanism;

public class Flywheel extends SubsystemBase {
    private FlywheelMechanism<?> io;

    public Flywheel(FlywheelMechanism<?> io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.periodic();
    }

    public void stop() {
        io.runVoltage(Volts.of(0.0));
    }

    public Command stopCommand() {
        return Commands.runOnce(this::stop, this);
    }

    public void start() {
        io.runVoltage(Volts.of(5.0));
    }

    public Command startCommand() {
        return Commands.runOnce(this::start, this);
    }
}
