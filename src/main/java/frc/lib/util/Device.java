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

package frc.lib.util;

import java.util.HashMap;
import java.util.HashSet;

/**
 * Sealed interface representing different types of hardware devices on the robot. Provides
 * type-safe device port definitions for CAN, DIO, and PWM devices.
 */
public sealed interface Device {
    /**
     * CAN bus device with an ID and bus name.
     *
     * @param id the CAN ID of the device
     * @param bus the name of the CAN bus the device is connected to (e.g., "rio", "Drivetrain")
     */
    final record CAN(int id, String bus) implements Device {
        // bus -> { ids }
        private static HashMap<String, HashSet<Integer>> ids = new HashMap<>();

        public CAN(int id, String bus) {
            if (id <= 0 || id > 63) {
                throw new IllegalArgumentException(
                        "Illegal CAN ID "
                                + id
                                + ". IDs must be between 1 and 63 inclusive (ID 0 is reserved).");
            }

            if (!ids.containsKey(bus)) {
                HashSet<Integer> idSet = new HashSet<>();
                idSet.add(id);
                ids.put(bus, idSet);
            } else if (ids.get(bus).contains(id)) {
                throw new IllegalArgumentException("CAN ID Conflict on ID " + id);
            } else {
                ids.get(bus).add(id);
            }

            this.id = id;
            this.bus = bus;
        }
    }

    /**
     * Digital Input/Output device with a port ID.
     *
     * @param id the DIO port ID on the robot controller
     */
    public final record DIO(int id) implements Device {}

    /**
     * Pulse Width Modulation device with a port ID.
     *
     * @param id the PWM port ID on the robot controller
     */
    public final record PWM(int id) implements Device {}

    /**
     * USB serial device (a USB-to-serial adapter such as the Pepper C1 RFID reader).
     *
     * <p>Systemcore has 4 USB-A ports, but WPILib 2027's {@code SerialPort} still only names USB 1
     * and 2. If that changes (or we switch to opening {@code /dev/tty*} paths directly), widen or
     * replace the range check here. See {@code SerialIOUSB} for the full list.
     *
     * @param id which USB serial port the device is on, 1 or 2 (WPILib only exposes two)
     */
    public final record USB(int id) implements Device {
        public USB {
            if (id < 1 || id > 2) {
                throw new IllegalArgumentException(
                        "Illegal USB serial port " + id + ". Ports must be 1 or 2.");
            }
        }
    }
}
