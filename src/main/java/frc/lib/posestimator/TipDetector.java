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

/**
 * Decides whether the robot is tipped, from its pitch and roll.
 *
 * <p>While tipped, wheels lose contact and slip, so odometry is unreliable, and a camera mounted
 * above the floor reports a pose that is displaced by about {@code height * sin(tilt)} once the
 * robot is projected onto the floor. A short hold after the tilt ends covers the robot settling,
 * because a frame captured as it comes back down is still suspect.
 *
 * <p>Ordinary driving gives a few degrees of pitch and roll, so the threshold has to be well above
 * that (3 degrees would flag about 15% of a match).
 */
public final class TipDetector {
    private double lastTiltedSeconds = Double.NEGATIVE_INFINITY;
    private double tiltDegrees = 0.0;

    /**
     * Reports whether the robot is tipped at {@code timestampSeconds}.
     *
     * @param timestampSeconds time of the pitch and roll sample
     * @param pitchDegrees robot pitch
     * @param rollDegrees robot roll
     * @param thresholdDegrees tilt, in either axis, above which the robot counts as tipped
     * @param holdSeconds how long the robot keeps counting as tipped after the tilt ends
     */
    public boolean update(
            double timestampSeconds,
            double pitchDegrees,
            double rollDegrees,
            double thresholdDegrees,
            double holdSeconds) {
        double pitch = Double.isFinite(pitchDegrees) ? Math.abs(pitchDegrees) : 0.0;
        double roll = Double.isFinite(rollDegrees) ? Math.abs(rollDegrees) : 0.0;
        tiltDegrees = Math.max(pitch, roll);

        boolean tilted = tiltDegrees > thresholdDegrees;
        if (tilted) {
            lastTiltedSeconds = timestampSeconds;
        }
        return tilted || timestampSeconds - lastTiltedSeconds < holdSeconds;
    }

    /** The larger of |pitch| and |roll| at the last update, in degrees. */
    public double tiltDegrees() {
        return tiltDegrees;
    }
}
