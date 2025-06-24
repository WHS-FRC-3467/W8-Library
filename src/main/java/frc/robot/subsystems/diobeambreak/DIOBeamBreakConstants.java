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

package frc.robot.subsystems.diobeambreak;

import frc.lib.io.beambreak.BeamBreakIO;
import frc.lib.io.beambreak.BeamBreakIODIO;
import frc.lib.io.beambreak.BeamBreakIOSim;
import frc.robot.Ports;

public class DIOBeamBreakConstants {
    
    public final static String NAME = "DIO Beam Break";
    public final static double DEBOUNCE_TIME = 0.1; // Seconds

    public static BeamBreakIODIO getReal()
    {
        return new BeamBreakIODIO(Ports.diobeambreak, NAME);
    }

    public static BeamBreakIOSim getSim()
    {
        return new BeamBreakIOSim(NAME);
    }

    public static BeamBreakIO getReplay()
    {
        return new BeamBreakIO() {};
    }
}
