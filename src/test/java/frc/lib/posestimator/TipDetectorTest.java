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

package frc.lib.posestimator;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class TipDetectorTest {
    private static final double THRESHOLD = 8.0;
    private static final double HOLD = 0.25;

    private final TipDetector detector = new TipDetector();

    private boolean update(double t, double pitch, double roll) {
        return detector.update(t, pitch, roll, THRESHOLD, HOLD);
    }

    @Test
    void ordinaryDrivingWobbleIsNotTipping() {
        assertFalse(update(0.0, 3.2, -2.9));
        assertFalse(update(0.1, -3.3, 3.1));
    }

    @Test
    void tiltPastTheThresholdInEitherAxisIsTipping() {
        assertTrue(update(0.0, 9.0, 0.0));
        assertTrue(update(10.0, 0.0, -9.0));
        assertTrue(update(20.0, -12.0, 2.0));
    }

    @Test
    void exactlyAtTheThresholdIsNotYetTipping() {
        assertFalse(update(0.0, 8.0, 0.0));
    }

    @Test
    void staysTippedForTheHoldAfterTheTiltEnds() {
        assertTrue(update(1.0, 10.0, 0.0));

        assertTrue(update(1.1, 0.5, 0.5), "still settling");
        assertTrue(update(1.24, 0.5, 0.5), "just inside the hold");
        assertFalse(update(1.25, 0.5, 0.5), "hold over");
        assertFalse(update(2.0, 0.5, 0.5));
    }

    @Test
    void aNewTiltRestartsTheHold() {
        update(1.0, 10.0, 0.0);
        update(1.2, 10.0, 0.0);

        assertTrue(update(1.4, 0.0, 0.0), "0.2 s after the latest tilt");
        assertFalse(update(1.5, 0.0, 0.0), "0.3 s after the latest tilt");
    }

    @Test
    void aZeroHoldMeansTippedOnlyWhileTilted() {
        assertTrue(detector.update(1.0, 10.0, 0.0, THRESHOLD, 0.0));
        assertFalse(detector.update(1.001, 0.0, 0.0, THRESHOLD, 0.0));
    }

    @Test
    void aBadReadingIsNotTipping() {
        assertFalse(update(0.0, Double.NaN, Double.POSITIVE_INFINITY));
    }

    @Test
    void reportsTheLargerTilt() {
        update(0.0, -4.0, 6.5);

        assertEquals(6.5, detector.tiltDegrees(), 1e-12);
    }
}
