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

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Seconds;

import frc.lib.io.serial.SerialIO;
import frc.lib.io.serial.SerialIOPepperC1Sim;
import java.util.Map;
import java.util.Optional;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.system.Timer;

class BatteryReaderTest {
    static final Map<String, String> KNOWN = Map.of("54D4F82A", "Battery 7");

    /** What the simulated reader has in range; tests change it. */
    String tagInRange = "";

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize()); // initialize the HAL, crash if failed
    }

    /** The reader made by the current test, closed afterwards so its alert IDs can be reused. */
    BatteryReader current;

    @AfterEach
    void cleanup() {
        if (current != null) {
            current.close();
        }
    }

    BatteryReader newReader(SerialIO io) {
        current = new BatteryReader("TestBattery", io, KNOWN, Seconds.of(10.0));
        return current;
    }

    /** Runs the reader's periodic on a 20 ms loop for the given time. */
    static void run(BatteryReader reader, double seconds) {
        for (int i = 0; i < seconds / 0.02; i++) {
            reader.periodic();
            Timer.delay(0.02);
        }
    }

    @Test
    void testReadsKnownBattery() {
        tagInRange = "54d4f82a"; // lowercase on purpose; the reader reports uppercase
        BatteryReader reader = newReader(new SerialIOPepperC1Sim(() -> tagInRange));

        run(reader, 0.6);

        assertEquals(Optional.of("54D4F82A"), reader.getUid(), "The tag's UID should be read");
        assertEquals(Optional.of("Battery 7"), reader.getBatteryName(), "A known UID should map to its battery name");
    }

    @Test
    void testUnknownBatteryHasUidButNoName() {
        tagInRange = "01020304";
        BatteryReader reader = newReader(new SerialIOPepperC1Sim(() -> tagInRange));

        run(reader, 0.6);

        assertEquals(Optional.of("01020304"), reader.getUid(), "The tag's UID should be read");
        assertTrue(reader.getBatteryName().isEmpty(), "An unknown UID should have no name");
    }

    @Test
    void testNoTagInRange() {
        BatteryReader reader = newReader(new SerialIOPepperC1Sim(() -> tagInRange));

        run(reader, 0.6);

        assertTrue(reader.getUid().isEmpty(), "No UID should be read when no tag is in range");
    }

    @Test
    void testFirstReadIsLockedIn() {
        tagInRange = "54D4F82A";
        BatteryReader reader = newReader(new SerialIOPepperC1Sim(() -> tagInRange));
        run(reader, 0.6);

        tagInRange = "01020304"; // tag "changes" without a reboot
        run(reader, 0.6);

        assertEquals(Optional.of("54D4F82A"), reader.getUid(), "The first tag read after boot should stay locked in");
    }

    @Test
    void testReplayNoOpIoDoesNotCrash() {
        BatteryReader reader = newReader(new SerialIO() {});

        run(reader, 0.3);

        assertTrue(reader.getUid().isEmpty(), "A no-op IO should never produce a UID");
    }
}
