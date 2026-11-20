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

import static org.wpilib.units.Units.Seconds;

import frc.lib.util.Device;

import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.hardware.bus.SerialPort;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Time;

/**
 * A {@link SerialIO} for a device plugged into a USB port through a USB-to-serial adapter, using
 * WPILib's {@link SerialPort}. Settings are 8 data bits, no parity, 1 stop bit, no flow control,
 * which is what almost every hobby serial device uses.
 *
 * <p>If the device is unplugged, the port is reopened about once a second instead of every loop, so
 * a missing device costs the robot loop almost nothing.
 *
 * <h2>Systemcore: still to check on real hardware</h2>
 *
 * <p>The code is ported to WPILib 2027, but none of this has run on a Systemcore yet:
 *
 * <ol>
 *   <li><b>Does it work at all?</b> Not confirmed that WPILib's {@code SerialPort} reaches a
 *       USB-to-serial adapter on Systemcore. Test it first. If it doesn't, replace the {@code
 *       SerialPort} calls here with jSerialComm (a Maven dependency) opening the Linux device
 *       directly. Find its name over SSH with {@code ls /dev/tty*} (likely {@code /dev/ttyUSB0} or
 *       {@code /dev/ttyACM0}).
 *   <li><b>Hotplug:</b> WPILib doesn't reconnect USB serial devices on Systemcore yet (allwpilib
 *       issue #8635). The reopen-once-a-second logic here is our workaround; keep it.
 *   <li><b>Permissions:</b> robot code runs as the {@code systemcore} Linux user, which may not be
 *       allowed to open serial devices. If opening fails with a permission error, that user needs
 *       to be in the {@code dialout} group (or a udev rule). Unverified.
 * </ol>
 */
public class SerialIOUSB implements SerialIO {
    private static final Time REOPEN_PERIOD = Seconds.of(1.0);

    private final String name;
    private final int baudRate;
    private final SerialPort.Port portId;

    private SerialPort port;
    private double lastOpenAttemptSeconds = Double.NEGATIVE_INFINITY;
    private boolean reportedFailure = false;

    /**
     * Constructs a {@link SerialIOUSB}. The port is opened on the first {@link #updateInputs} call.
     *
     * @param name A human readable name for this device, used in error messages
     * @param usb The USB serial port the device is plugged into
     * @param baudRate The device's baud rate, in bits per second (e.g. 115200)
     */
    public SerialIOUSB(String name, Device.USB usb, int baudRate) {
        this.name = name;
        this.baudRate = baudRate;
        this.portId = usb.id() == 1 ? SerialPort.Port.USB_1 : SerialPort.Port.USB_2;
    }

    @Override
    public void updateInputs(SerialInputs inputs) {
        if (port == null) {
            tryOpen();
        }

        inputs.received = new byte[0];
        if (port != null) {
            try {
                int available = port.getBytesReceived();
                if (available > 0) {
                    inputs.received = port.read(available);
                }
            } catch (RuntimeException e) {
                // WPILib throws if the adapter was unplugged. Close and let tryOpen retry.
                reportFailure(e);
                close();
            }
        }
        inputs.connected = port != null;
    }

    @Override
    public void write(byte[] data) {
        if (port == null) {
            return;
        }
        try {
            port.write(data, data.length);
        } catch (RuntimeException e) {
            reportFailure(e);
            close();
        }
    }

    @Override
    public void close() {
        if (port != null) {
            port.close();
            port = null;
        }
    }

    private void tryOpen() {
        double now = Timer.getTimestamp();
        if (now - lastOpenAttemptSeconds < REOPEN_PERIOD.in(Seconds)) {
            return;
        }
        lastOpenAttemptSeconds = now;

        try {
            port = new SerialPort(baudRate, portId);
            port.reset(); // throw away anything left in the buffer from before we opened it
        } catch (RuntimeException e) {
            reportFailure(e);
            port = null;
        }
    }

    private void reportFailure(RuntimeException e) {
        // Only report once so an unplugged device doesn't flood the console.
        if (!reportedFailure) {
            reportedFailure = true;
            DriverStationErrors.reportWarning(
                    name + ": serial port " + portId + " failed: " + e.getMessage(), false);
        }
    }
}
