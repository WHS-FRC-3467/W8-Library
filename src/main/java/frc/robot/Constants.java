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

package frc.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;

import org.wpilib.framework.RobotBase;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final Mode SIM_MODE = Mode.SIM;
    public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

    public static final boolean TUNING_MODE = true;
    public static final double LOOP_PERIOD = 1.0 / 50.0;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,
        /** Running a physics simulator. */
        SIM,
        /** Replaying from a log file. */
        REPLAY
    }

    public static final class PathConstants {}

    public static final Distance STARTING_POSE_DRIVE_TOLERANCE = Inches.of(3.0); // For auto
    public static final Angle STARTING_POSE_ROT_TOLERANCE_DEGREES = Degrees.of(5.0);

    public static final Distance FULL_ROBOT_WIDTH = Inches.of(27.0 + 3.25);
    public static final Distance FULL_ROBOT_LENGTH = Inches.of(27.0 + 3.25);
    public static final Distance BUMPER_HEIGHT = Inches.of(4.0);
}
