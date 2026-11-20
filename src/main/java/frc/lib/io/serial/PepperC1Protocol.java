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

import lombok.AccessLevel;
import lombok.NoArgsConstructor;

import java.util.Arrays;
import java.util.HexFormat;
import java.util.Optional;
import java.util.OptionalInt;

/**
 * Encodes and decodes frames for the Eccel Pepper C1 RFID reader's binary protocol. This class
 * touches no hardware, so it can be unit tested on its own.
 *
 * <p>Every frame, in both directions, looks like this (C1 software manual, section 5.2):
 *
 * <pre>
 * 0xF5 | len LSB, len MSB | (len XOR 0xFFFF) LSB, MSB | body ... | CRC16 LSB, MSB
 * </pre>
 *
 * <p>{@code len} counts the body plus the 2 CRC bytes. The CRC covers only the body. The second
 * copy of the length (inverted) lets us reject a corrupted header before trusting it.
 */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public final class PepperC1Protocol {
    /** First byte of every frame. */
    public static final int STX = 0xF5;

    /** Number of bytes after STX that describe the frame length. */
    public static final int LENGTH_BYTES = 4;

    /** Longest {@code len} the reader allows (manual section 5.2). */
    public static final int MAX_LENGTH = 1024;

    /** First body byte of a successful reply. */
    public static final int ACK = 0x00;

    /** Command: count the tags in range and remember their UIDs inside the reader. */
    public static final int GET_TAG_COUNT = 0x02;

    /** Command: read the UID of one tag found by {@link #GET_TAG_COUNT}. */
    public static final int GET_TAG_UID = 0x03;

    private static final HexFormat HEX = HexFormat.of().withUpperCase();

    /**
     * Builds a complete frame around a command body.
     *
     * @param body the command ID followed by its parameters
     * @return the bytes to write to the serial port
     */
    public static byte[] encode(byte... body) {
        int length = body.length + 2;
        int invertedLength = length ^ 0xFFFF;
        int crc = crc16(body);

        byte[] frame = new byte[1 + LENGTH_BYTES + length];
        frame[0] = (byte) STX;
        frame[1] = (byte) length;
        frame[2] = (byte) (length >> 8);
        frame[3] = (byte) invertedLength;
        frame[4] = (byte) (invertedLength >> 8);
        System.arraycopy(body, 0, frame, 5, body.length);
        frame[frame.length - 2] = (byte) crc;
        frame[frame.length - 1] = (byte) (crc >> 8);
        return frame;
    }

    /**
     * Reads the length out of the 4 bytes that follow STX.
     *
     * @param lengthBytes the 4 bytes after STX
     * @return how many more bytes to read (body + CRC), or empty if the header is corrupted
     */
    public static OptionalInt decodeLength(byte[] lengthBytes) {
        if (lengthBytes.length != LENGTH_BYTES) {
            return OptionalInt.empty();
        }
        int length = (lengthBytes[0] & 0xFF) | ((lengthBytes[1] & 0xFF) << 8);
        int inverted = (lengthBytes[2] & 0xFF) | ((lengthBytes[3] & 0xFF) << 8);
        if ((length ^ 0xFFFF) != inverted || length < 2 || length > MAX_LENGTH) {
            return OptionalInt.empty();
        }
        return OptionalInt.of(length);
    }

    /**
     * Checks the CRC on the rest of a frame and returns its body.
     *
     * @param bodyAndCrc the {@code len} bytes after the length header
     * @return the body without the CRC, or empty if the CRC does not match
     */
    public static Optional<byte[]> decodeBody(byte[] bodyAndCrc) {
        if (bodyAndCrc.length < 2) {
            return Optional.empty();
        }
        byte[] body = Arrays.copyOf(bodyAndCrc, bodyAndCrc.length - 2);
        int receivedCrc =
                (bodyAndCrc[bodyAndCrc.length - 2] & 0xFF)
                        | ((bodyAndCrc[bodyAndCrc.length - 1] & 0xFF) << 8);
        return crc16(body) == receivedCrc ? Optional.of(body) : Optional.empty();
    }

    /**
     * Whether a reply body is a successful answer to the given command. The reader can also send
     * unrequested frames (for example if its built-in polling mode is left on), so callers should
     * skip bodies that fail this check.
     *
     * @param body a decoded reply body
     * @param command the command we sent
     * @return true if the body starts with ACK followed by that command ID
     */
    public static boolean isAckFor(byte[] body, int command) {
        return body.length >= 2 && (body[0] & 0xFF) == ACK && (body[1] & 0xFF) == command;
    }

    /**
     * Parses a reply to {@link #GET_TAG_COUNT}.
     *
     * @param body a decoded reply body
     * @return the number of tags in range, or empty if this is not a tag count reply
     */
    public static OptionalInt parseTagCount(byte[] body) {
        if (!isAckFor(body, GET_TAG_COUNT) || body.length < 3) {
            return OptionalInt.empty();
        }
        return OptionalInt.of(body[2] & 0xFF);
    }

    /**
     * Parses a reply to {@link #GET_TAG_UID}. The body is ACK, command, tag type, SAK/DSFID, then 4
     * to 8 UID bytes.
     *
     * @param body a decoded reply body
     * @return the UID as uppercase hex (e.g. {@code "54D4F82A"}), or empty if this is not a UID
     *     reply
     */
    public static Optional<String> parseTagUid(byte[] body) {
        if (!isAckFor(body, GET_TAG_UID) || body.length < 5) {
            return Optional.empty();
        }
        return Optional.of(HEX.formatHex(body, 4, body.length));
    }

    /**
     * Collects bytes as they arrive over several robot loops and splits them into frames. A frame
     * can arrive in pieces, and noise or a half-sent frame can show up before it, so this keeps
     * unfinished bytes until the rest arrive and skips anything that isn't a valid frame.
     */
    public static final class FrameReader {
        /** Enough for several maximum-size frames; older bytes are dropped past this. */
        private static final int MAX_BUFFERED_BYTES = 4 * (1 + LENGTH_BYTES + MAX_LENGTH);

        private byte[] buffer = new byte[0];

        /**
         * Adds newly received bytes.
         *
         * @param received bytes in the order they arrived
         */
        public void add(byte[] received) {
            byte[] combined = Arrays.copyOf(buffer, buffer.length + received.length);
            System.arraycopy(received, 0, combined, buffer.length, received.length);
            int start = Math.max(0, combined.length - MAX_BUFFERED_BYTES);
            buffer = Arrays.copyOfRange(combined, start, combined.length);
        }

        /**
         * Removes and returns the next complete, valid frame body.
         *
         * @return the body, or empty if no complete frame has arrived yet
         */
        public Optional<byte[]> next() {
            while (true) {
                int stx = indexOfStx();
                if (stx < 0) {
                    buffer = new byte[0];
                    return Optional.empty();
                }
                buffer = Arrays.copyOfRange(buffer, stx, buffer.length);
                if (buffer.length < 1 + LENGTH_BYTES) {
                    return Optional.empty(); // wait for the rest of the header
                }

                OptionalInt length = decodeLength(Arrays.copyOfRange(buffer, 1, 1 + LENGTH_BYTES));
                if (length.isEmpty()) {
                    // This 0xF5 was a data byte, not a real frame start. Skip it and look again.
                    buffer = Arrays.copyOfRange(buffer, 1, buffer.length);
                    continue;
                }

                int frameEnd = 1 + LENGTH_BYTES + length.getAsInt();
                if (buffer.length < frameEnd) {
                    return Optional.empty(); // wait for the rest of the frame
                }
                Optional<byte[]> body =
                        decodeBody(Arrays.copyOfRange(buffer, 1 + LENGTH_BYTES, frameEnd));
                buffer = Arrays.copyOfRange(buffer, frameEnd, buffer.length);
                if (body.isPresent()) {
                    return body;
                }
                // Bad CRC: drop the frame and keep looking.
            }
        }

        private int indexOfStx() {
            for (int i = 0; i < buffer.length; i++) {
                if ((buffer[i] & 0xFF) == STX) {
                    return i;
                }
            }
            return -1;
        }
    }

    /**
     * CRC-16/CCITT as the reader expects it: polynomial 0x1021, starting value 0xFFFF, no final XOR
     * (manual section 5.3).
     *
     * @param data the bytes to check
     * @return the 16-bit CRC
     */
    public static int crc16(byte[] data) {
        int crc = 0xFFFF;
        for (byte b : data) {
            crc ^= (b & 0xFF) << 8;
            for (int bit = 0; bit < 8; bit++) {
                crc = (crc & 0x8000) != 0 ? (crc << 1) ^ 0x1021 : crc << 1;
                crc &= 0xFFFF;
            }
        }
        return crc;
    }
}
