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

package frc.lib.io.serial;

import org.littletonrobotics.junction.AutoLog;

/**
 * Standardized interface for any serial (UART / USB serial) device used in FRC.
 *
 * <p>This layer only moves raw bytes. What the bytes mean (frames, checksums, commands) belongs to
 * the device that owns the IO, so the same IO works for an RFID reader, an LED controller, or
 * anything else with a serial port. Because received bytes are logged as inputs, AdvantageKit
 * replay feeds the device the exact bytes it saw on the real robot.
 */
public interface SerialIO extends AutoCloseable {

    @AutoLog
    abstract class SerialInputs {
        /** Whether the serial port is open. This doesn't prove the device on it is answering. */
        public boolean connected = false;

        /** Bytes received since the last update, oldest first. Empty if nothing arrived. */
        public byte[] received = new byte[0];
    }

    /**
     * Updates the provided {@link SerialInputs} instance with every byte received since the last
     * call. This must not block; it only returns what has already arrived.
     *
     * @param inputs The structure to populate with updated serial data.
     */
    public default void updateInputs(SerialInputs inputs) {}

    /**
     * Sends bytes to the device. Does nothing if the port is not open.
     *
     * @param data The bytes to send.
     */
    public default void write(byte[] data) {}

    /** Releases the serial port. */
    @Override
    public default void close() {}
}
