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

import java.io.ByteArrayOutputStream;
import java.util.HexFormat;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

/**
 * Pretends to be a Pepper C1 RFID reader on a serial port. It answers the same binary commands the
 * real reader does, so the code talking to it runs exactly as it would on the robot.
 *
 * <p>In simulation, type a tag UID in hex (e.g. {@code 54D4F82A}) into the dashboard field {@code
 * <name>/SimUID} to "insert" that battery. Leave it blank for no tag.
 */
public class SerialIOPepperC1Sim implements SerialIO {
    /** Tag type the sim reports: MIFARE Classic 1k (manual section 10.1.5). */
    private static final int SIM_TAG_TYPE = 0x04;

    /** SAK byte for a MIFARE Classic 1k tag. */
    private static final int SIM_TAG_SAK = 0x08;

    private final Supplier<String> uidSupplier;
    private final PepperC1Protocol.FrameReader commandReader = new PepperC1Protocol.FrameReader();
    private final ByteArrayOutputStream pendingReplies = new ByteArrayOutputStream();

    /**
     * Constructs a sim reader whose tag is set from the dashboard.
     *
     * @param name A human readable name for this reader, used as the dashboard key prefix
     */
    public SerialIOPepperC1Sim(String name) {
        this(new LoggedNetworkString(name + "/SimUID", "")::get);
    }

    /**
     * Constructs a sim reader whose tag comes from code. Useful in unit tests.
     *
     * @param uidSupplier Supplies the UID in range as hex, or "" for no tag
     */
    public SerialIOPepperC1Sim(Supplier<String> uidSupplier) {
        this.uidSupplier = uidSupplier;
    }

    @Override
    public void updateInputs(SerialInputs inputs) {
        inputs.connected = true;
        inputs.received = pendingReplies.toByteArray();
        pendingReplies.reset();
    }

    @Override
    public void write(byte[] data) {
        commandReader.add(data);
        for (Optional<byte[]> command = commandReader.next(); command.isPresent(); command = commandReader.next()) {
            respondTo(command.get());
        }
    }

    private void respondTo(byte[] command) {
        if (command.length == 0) {
            return;
        }
        Optional<byte[]> uid = currentUid();
        switch (command[0] & 0xFF) {
            case PepperC1Protocol.GET_TAG_COUNT ->
                reply(PepperC1Protocol.ACK, PepperC1Protocol.GET_TAG_COUNT, uid.isPresent() ? 1 : 0);
            case PepperC1Protocol.GET_TAG_UID -> {
                if (uid.isEmpty()) {
                    // Real reader answers with an error frame here; any non-ACK first byte works.
                    reply(0x01, PepperC1Protocol.GET_TAG_UID);
                    return;
                }
                byte[] body = new byte[4 + uid.get().length];
                body[0] = (byte) PepperC1Protocol.ACK;
                body[1] = (byte) PepperC1Protocol.GET_TAG_UID;
                body[2] = (byte) SIM_TAG_TYPE;
                body[3] = (byte) SIM_TAG_SAK;
                System.arraycopy(uid.get(), 0, body, 4, uid.get().length);
                pendingReplies.writeBytes(PepperC1Protocol.encode(body));
            }
            default -> reply(0x01, command[0] & 0xFF); // "command not supported"
        }
    }

    private void reply(int... body) {
        byte[] bytes = new byte[body.length];
        for (int i = 0; i < body.length; i++) {
            bytes[i] = (byte) body[i];
        }
        pendingReplies.writeBytes(PepperC1Protocol.encode(bytes));
    }

    /** The UID from the supplier as bytes, or empty if it is blank or not valid hex. */
    private Optional<byte[]> currentUid() {
        String hex = uidSupplier.get().trim();
        if (hex.isEmpty()) {
            return Optional.empty();
        }
        try {
            return Optional.of(HexFormat.of().parseHex(hex));
        } catch (IllegalArgumentException e) {
            return Optional.empty(); // half-typed or odd-length text on the dashboard
        }
    }
}
