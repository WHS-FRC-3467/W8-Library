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

import frc.lib.io.serial.SerialIO;
import frc.lib.sensors.BatteryReader;

import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;

import java.util.Optional;

/** Reads which battery is in the robot at boot from the RFID tag stuck on it. */
public class Battery implements Mechanism {
    private final BatteryReader reader;

    /**
     * Constructs the Battery subsystem.
     *
     * @param io the serial port the RFID reader is on
     */
    public Battery(SerialIO io) {
        reader =
                new BatteryReader(
                        BatteryConstants.NAME,
                        io,
                        BatteryConstants.KNOWN_BATTERIES,
                        BatteryConstants.READ_TIMEOUT);

        Scheduler.getDefault().addPeriodic(this::periodic);
    }

    /** Talks to the RFID reader and logs the result. Runs every loop via the scheduler. */
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
