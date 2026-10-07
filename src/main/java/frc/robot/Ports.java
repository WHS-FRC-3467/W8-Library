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

package frc.robot;

import com.ctre.phoenix6.CANBus;

import frc.lib.util.Device;
import frc.lib.util.Device.CAN;

public class Ports {
    /*
     * LIST OF CHANNEL AND CAN IDS
     */
    public static final CANBus DRIVETRAIN_BUS = new CANBus("Drivetrain");

    public static final Device.CAN flywheel = new CAN(2, "rio");
}
