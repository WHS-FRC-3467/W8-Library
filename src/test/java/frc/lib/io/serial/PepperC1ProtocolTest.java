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

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.HexFormat;
import java.util.Optional;

class PepperC1ProtocolTest {
    /** Example frame from the C1 software manual, section 9.1.5: an async GET_UID report. */
    static final byte[] MANUAL_FRAME = hex("F50A00F5FFFE03010854D4F82A7364");

    static final byte[] MANUAL_BODY = hex("FE03010854D4F82A");

    static byte[] hex(String text) {
        return HexFormat.of().parseHex(text);
    }

    @Test
    void testCrcMatchesManualExample() {
        assertEquals(0x6473, PepperC1Protocol.crc16(MANUAL_BODY),
                "CRC of the manual's example body should be 0x6473 (sent as 73 64)");
    }

    @Test
    void testEncodeMatchesManualExample() {
        assertArrayEquals(MANUAL_FRAME, PepperC1Protocol.encode(MANUAL_BODY),
                "Encoding the manual's example body should produce the manual's example frame");
    }

    @Test
    void testEncodeGetTagCount() {
        // len = 1 body byte + 2 CRC = 3, inverted = 0xFFFC, CRC of {02} = 0xC1B2
        assertArrayEquals(hex("F50300FCFF02B2C1"),
                PepperC1Protocol.encode((byte) PepperC1Protocol.GET_TAG_COUNT),
                "GET_TAG_COUNT frame should have the expected header and CRC");
    }

    @Test
    void testDecodeLengthRejectsBadInverse() {
        assertTrue(PepperC1Protocol.decodeLength(hex("0A00F5FF")).isPresent(),
                "A correct length header should decode");
        assertTrue(PepperC1Protocol.decodeLength(hex("0A00F5FE")).isEmpty(),
                "A length header whose inverted copy doesn't match should be rejected");
    }

    @Test
    void testFrameReaderHandlesPiecesAndJunk() {
        PepperC1Protocol.FrameReader reader = new PepperC1Protocol.FrameReader();

        // Junk (including a stray 0xF5) before the frame, and the frame split across two reads.
        reader.add(hex("0011F522"));
        reader.add(Arrays.copyOfRange(MANUAL_FRAME, 0, 7));
        assertTrue(reader.next().isEmpty(), "Half a frame should not decode yet");

        reader.add(Arrays.copyOfRange(MANUAL_FRAME, 7, MANUAL_FRAME.length));
        Optional<byte[]> body = reader.next();
        assertTrue(body.isPresent(), "The frame should decode once all of it has arrived");
        assertArrayEquals(MANUAL_BODY, body.get(), "The decoded body should match the manual");
        assertTrue(reader.next().isEmpty(), "There should be no second frame");
    }

    @Test
    void testFrameReaderDropsBadCrc() {
        PepperC1Protocol.FrameReader reader = new PepperC1Protocol.FrameReader();
        byte[] corrupted = MANUAL_FRAME.clone();
        corrupted[8] ^= 0x01; // flip a bit in the UID

        reader.add(corrupted);
        reader.add(MANUAL_FRAME);

        Optional<byte[]> body = reader.next();
        assertTrue(body.isPresent(), "The good frame after the corrupted one should decode");
        assertArrayEquals(MANUAL_BODY, body.get(), "Only the uncorrupted frame should come out");
        assertTrue(reader.next().isEmpty(), "The corrupted frame should have been dropped");
    }

    @Test
    void testParseRepliesFromManual() {
        // Section 10.1.4: ACK, GET_TAG_COUNT, 1 tag
        assertEquals(1, PepperC1Protocol.parseTagCount(hex("000201")).getAsInt(),
                "Tag count reply should report 1 tag");
        // Section 10.1.5: ACK, GET_TAG_UID, type, SAK, UID 74 54 12 65
        assertEquals("74541265", PepperC1Protocol.parseTagUid(hex("0003012074541265")).get(),
                "UID reply should parse to uppercase hex");
        assertTrue(PepperC1Protocol.parseTagUid(hex("0103")).isEmpty(),
                "An error reply should not parse as a UID");
    }
}
