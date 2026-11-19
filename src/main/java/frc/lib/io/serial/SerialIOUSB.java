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

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

import frc.lib.util.Device;

/**
 * A {@link SerialIO} for a device plugged into a USB port through a USB-to-serial adapter, using
 * WPILib's {@link SerialPort}. Settings are 8 data bits, no parity, 1 stop bit, no flow control,
 * which is what almost every hobby serial device uses.
 *
 * <p>If the device is unplugged, the port is reopened about once a second instead of every loop,
 * so a missing device costs the robot loop almost nothing.
 *
 * <h2>Moving to WPILib 2027 / Systemcore</h2>
 *
 * <p>This is the only serial file that touches WPILib hardware, so most of the port happens here.
 * Everything below was checked against the 2027 alphas (Nov 2026); re-check before relying on it.
 *
 * <ol>
 *   <li><b>Imports:</b> {@code edu.wpi.first.wpilibj.SerialPort} becomes {@code
 *       org.wpilib.hardware.bus.SerialPort}. Enum values are ALL_CAPS in 2027, so {@code
 *       Port.kUSB1}/{@code kUSB2} become {@code Port.USB_1}/{@code USB_2}.
 *   <li><b>Does it work at all?</b> Not confirmed: the 2027 {@code SerialPort} still has USB ports,
 *       but nobody has confirmed it reaches a USB-to-serial adapter on Systemcore. Test it first. If
 *       it doesn't, replace the {@code SerialPort} calls here with jSerialComm (a Maven dependency)
 *       opening the Linux device directly. Find its name over SSH with {@code ls /dev/tty*} (likely
 *       {@code /dev/ttyUSB0} or {@code /dev/ttyACM0}).
 *   <li><b>Hotplug:</b> WPILib doesn't reconnect USB serial devices on Systemcore yet (allwpilib
 *       issue #8635). The reopen-once-a-second logic here is our workaround; keep it.
 *   <li><b>Permissions:</b> robot code runs as the {@code systemcore} Linux user, which may not be
 *       allowed to open serial devices. If opening fails with a permission error, that user needs
 *       to be in the {@code dialout} group (or a udev rule). Unverified.
 *   <li><b>Timestamps:</b> there is no FPGA on Systemcore. Check what {@code
 *       Timer.getFPGATimestamp()} became and switch to it.
 *   <li><b>Warnings:</b> {@code DriverStation} is split into {@code MatchState} and {@code
 *       RobotState} in 2027. Find where {@code reportWarning} went (or log through an Alert).
 * </ol>
 *
 * <p>Classes built on {@link SerialIO} ({@code BatteryReader}, {@code SerialIOPepperC1Sim}, {@code
 * PepperC1Protocol}) only need the usual import renames; the protocol itself doesn't change.
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
        this.portId = usb.id() == 1 ? SerialPort.Port.kUSB1 : SerialPort.Port.kUSB2;
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
        double now = Timer.getFPGATimestamp();
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
            DriverStation.reportWarning(
                    name + ": serial port " + portId + " failed: " + e.getMessage(), false);
        }
    }
}
