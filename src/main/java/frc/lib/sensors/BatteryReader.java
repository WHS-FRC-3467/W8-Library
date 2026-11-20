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

package frc.lib.sensors;

import static org.wpilib.units.Units.Seconds;

import frc.lib.io.serial.PepperC1Protocol;
import frc.lib.io.serial.SerialIO;
import frc.lib.io.serial.SerialInputsAutoLogged;

import org.littletonrobotics.junction.Logger;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Time;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

import java.util.Map;
import java.util.Optional;

/**
 * Identifies which battery is in the robot by reading an RFID tag stuck on it with an Eccel Pepper
 * C1 reader, then looks the tag up in a table of known batteries.
 *
 * <p>Swapping batteries means power-cycling the robot, so the first tag read after boot is locked
 * in and the reader is not asked again. Until then it asks every {@link #POLL_PERIOD}.
 *
 * <p>All protocol work happens here, on bytes from {@link SerialIO}. The IO only moves bytes, so in
 * AdvantageKit replay this class re-reads the same logged bytes and gets the same answer.
 *
 * <p>The reader needs its Wi-Fi and Bluetooth turned off in its web interface; robot rules don't
 * allow extra radios. Systemcore hardware notes live in {@code SerialIOUSB}.
 */
public class BatteryReader {
    /** How often to ask the reader for a tag. Matches the reader's own default polling period. */
    private static final Time POLL_PERIOD = Seconds.of(0.2);

    private final String name;
    private final SerialIO io;
    private final SerialInputsAutoLogged inputs = new SerialInputsAutoLogged();
    private final Map<String, String> knownBatteries;
    private final Time readTimeout;

    private final PepperC1Protocol.FrameReader frameReader = new PepperC1Protocol.FrameReader();
    private final Timer sinceStart = new Timer();
    private final Timer sincePoll = new Timer();

    private boolean readerResponded = false;
    private Optional<String> uid = Optional.empty();

    private final Alert notReadAlert;
    private final Alert unknownBatteryAlert;

    /**
     * Constructs a BatteryReader.
     *
     * @param name the name to use for logging and alerts
     * @param io the serial port the reader is on
     * @param knownBatteries tag UID (uppercase hex, e.g. "54D4F82A") to battery name
     * @param readTimeout how long after boot to wait for a tag before raising an alert
     */
    public BatteryReader(
            String name, SerialIO io, Map<String, String> knownBatteries, Time readTimeout) {
        this.name = name;
        this.io = io;
        this.knownBatteries = Map.copyOf(knownBatteries);
        this.readTimeout = readTimeout;

        notReadAlert = new Alert(name + "_battery_not_read", "", Level.MEDIUM);
        unknownBatteryAlert = new Alert(name + "_battery_unknown", "", Level.MEDIUM);

        sinceStart.start();
        sincePoll.start();
    }

    /** Call this method periodically to talk to the reader and log data. */
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);

        if (uid.isEmpty()) {
            handleReplies();
            if (uid.isEmpty() && sincePoll.advanceIfElapsed(POLL_PERIOD.in(Seconds))) {
                send(PepperC1Protocol.GET_TAG_COUNT);
            }
        }

        updateAlerts();
        Logger.recordOutput(name + "/ReaderResponded", readerResponded);
        Logger.recordOutput(name + "/UID", uid.orElse(""));
        Logger.recordOutput(name + "/BatteryName", getBatteryName().orElse(""));
    }

    /**
     * The UID of the battery's tag, once one has been read.
     *
     * @return the UID as uppercase hex, or empty if no tag has been read yet
     */
    public Optional<String> getUid() {
        return uid;
    }

    /**
     * The name of the battery in the robot, if its tag has been read and is in the known table.
     *
     * @return the battery name, or empty if not read yet or not a known battery
     */
    public Optional<String> getBatteryName() {
        return uid.map(knownBatteries::get);
    }

    /**
     * Closes the serial port and releases the alerts. WPILib 2027 alert IDs must be unique, so a
     * second reader with the same name can only be made after this one is closed.
     */
    public void close() {
        io.close();
        notReadAlert.close();
        unknownBatteryAlert.close();
    }

    private void handleReplies() {
        frameReader.add(inputs.received);
        for (Optional<byte[]> body = frameReader.next();
                body.isPresent() && uid.isEmpty();
                body = frameReader.next()) {
            readerResponded = true;

            // A tag is in range: ask for its UID right away instead of waiting for the next poll.
            PepperC1Protocol.parseTagCount(body.get())
                    .ifPresent(
                            count -> {
                                if (count > 0) {
                                    send(
                                            PepperC1Protocol.GET_TAG_UID,
                                            0x00); // tag 0: only one battery
                                }
                            });

            PepperC1Protocol.parseTagUid(body.get()).ifPresent(tag -> uid = Optional.of(tag));
        }
    }

    private void send(int... body) {
        byte[] bytes = new byte[body.length];
        for (int i = 0; i < body.length; i++) {
            bytes[i] = (byte) body[i];
        }
        io.write(PepperC1Protocol.encode(bytes));
    }

    private void updateAlerts() {
        boolean timedOut = uid.isEmpty() && sinceStart.hasElapsed(readTimeout.in(Seconds));
        if (!inputs.connected) {
            notReadAlert.setText(name + ": RFID reader not plugged in, battery unknown");
        } else if (!readerResponded) {
            notReadAlert.setText(name + ": RFID reader not answering, battery unknown");
        } else {
            notReadAlert.setText(name + ": no battery tag in range of the RFID reader");
        }
        notReadAlert.set(timedOut);

        unknownBatteryAlert.setText(
                name + ": battery tag " + uid.orElse("") + " is not in the known battery list");
        unknownBatteryAlert.set(uid.isPresent() && getBatteryName().isEmpty());
    }
}
