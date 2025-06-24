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

package frc.lib.io.powerdistribution;

import org.littletonrobotics.junction.AutoLog;

/** Interface for the Power Distribution Hub (REV PDH) or Panel (CTRE PDP) */
public interface PowerDistributionIO {
    
    @AutoLog
    abstract class PowerDistributionInputs {
        /** Voltage of the Power Distribution Hub/Panel in volts */
        public double voltage = 0.0;
        /** Temperature of the CTRE PDP in degrees Celsius. Not supported on the REV PDH so value will remain 0.0. */
        public double temperature = 0.0;
        /** Current of each channel of the PDP/PDH in Amperes */
        public double[] currents = null;
        /** Total current of the PDP/PDH in Amperes */
        public double totalCurrent = 0.0;
        /** Total power of the PDP in Watts. Not supported on the REV PDH so value will remain 0.0. */
        public double totalPower = 0.0;
        /** Total energy of the PDP in Joules. Not supported on the REV PDH so value will remain 0.0.*/
        public double totalEnergy = 0.0;
    }

    public default void updateInputs(PowerDistributionInputs inputs) {}
}
