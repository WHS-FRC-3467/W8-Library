/*
 * Copyright (C) 2025 Windham Windup
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

package frc.robot.subsystems.servo1;

import frc.lib.io.servo.ServoIO;
import frc.lib.io.servo.ServoIOPWM;
import frc.lib.io.servo.ServoIOSim;
import frc.robot.Ports;

public class Servo1Constants {

    public final static String NAME = "Servo #1";

    public final static double MINIMUM_ANGLE = 0.0;
    // Change as necessary
    public final static double MAXIMUM_ANGLE = 180.0;

    public static ServoIOPWM getReal()
    {
        return new ServoIOPWM(Ports.servo1, NAME, MINIMUM_ANGLE, MAXIMUM_ANGLE);
    }

    public static ServoIOSim getSim()
    {
        return new ServoIOSim(NAME, MINIMUM_ANGLE, MAXIMUM_ANGLE);
    }

    public static ServoIO getReplay()
    {
        return new ServoIO() {};
    }
}
