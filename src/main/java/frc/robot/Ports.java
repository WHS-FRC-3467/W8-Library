/* Copyright (C) 2025 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

package frc.robot;

import frc.lib.util.Device;
import frc.lib.util.Device.CAN;
import frc.lib.util.Device.DIO;

public class Ports {
    /*
     * LIST OF CHANNEL AND CAN IDS
     */

    public static final Device.CAN laserCAN1 = new CAN(0, "rio");
    public static final Device.CAN lights = new CAN(1, "rio");

    public static final Device.DIO diobeambreak = new DIO(2);

    public static final Device.CAN pdh = new CAN(50, "rio");

}
