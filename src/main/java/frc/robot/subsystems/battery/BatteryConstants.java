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

import static org.wpilib.units.Units.Seconds;

import frc.lib.io.serial.SerialIO;
import frc.lib.io.serial.SerialIOPepperC1Sim;
import frc.lib.io.serial.SerialIOUSB;
import frc.robot.Constants;
import frc.robot.Ports;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;

import org.wpilib.units.measure.Time;

import java.util.Map;

/** Configuration and factory for the {@link Battery} subsystem. */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class BatteryConstants {
    public static final String NAME = "Battery";

    /** The Pepper C1 reader's default baud rate (C1 software manual, section 5.1). */
    private static final int READER_BAUD_RATE = 115200;

    /** How long after boot to wait for a tag before warning the drive team. */
    public static final Time READ_TIMEOUT = Seconds.of(10.0);

    /**
     * Tag UID to battery name. To add a battery: stick a tag on it, put it in the robot, boot, read
     * {@code Battery/UID} from the log, and add a line here.
     */
    public static final Map<String, String> KNOWN_BATTERIES =
            Map.of("54D4F82A", "Example Battery 1");

    /**
     * Creates the Battery subsystem with IO implementations for the current robot mode.
     *
     * @return the configured Battery subsystem
     */
    public static Battery get() {
        return switch (Constants.CURRENT_MODE) {
            case REAL -> new Battery(new SerialIOUSB(NAME, Ports.batteryReader, READER_BAUD_RATE));
            case SIM -> new Battery(new SerialIOPepperC1Sim(NAME));
            case REPLAY -> new Battery(new SerialIO() {});
        };
    }
}
