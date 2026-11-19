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

package frc.robot.subsystems.battery;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.devices.BatteryReader;
import frc.lib.io.serial.SerialIO;

import java.util.Optional;

/** Reads which battery is in the robot at boot from the RFID tag stuck on it. */
public class Battery extends SubsystemBase {
    private final BatteryReader reader;

    /**
     * Constructs the Battery subsystem.
     *
     * @param io the serial port the RFID reader is on
     */
    public Battery(SerialIO io) {
        reader = new BatteryReader(
                BatteryConstants.NAME,
                io,
                BatteryConstants.KNOWN_BATTERIES,
                BatteryConstants.READ_TIMEOUT);
    }

    @Override
    public void periodic() {
        reader.periodic();
    }

    /**
     * The name of the battery in the robot.
     *
     * @return the battery name, or empty if its tag hasn't been read or isn't in the known list
     */
    public Optional<String> getBatteryName() {
        return reader.getBatteryName();
    }
}
