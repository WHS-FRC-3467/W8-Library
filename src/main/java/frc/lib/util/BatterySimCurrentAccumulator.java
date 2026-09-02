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
package frc.lib.util;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;

/** A utility class extending the default functionality of {@link BatterySim}. */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public final class BatterySimCurrentAccumulator {
    /** The current accumulated current draw (positive or negative) loading the battery. */
    private static Current simCurrentSum = Amps.zero();
    
    /**
     * Adds a mechanism's signed battery/supply current to the accumulator.
     *
     * <p>Positive current is treated as battery draw. Negative current is treated as
     * regeneration to the bus and reduces the net sag. Callers should pass signed
     * supply/battery current, not torque/stator current.
     *
     * @param current signed supply current
     */
    public static void addCurrentLoad(Current current) {
        simCurrentSum = simCurrentSum.plus(current);
    }

    /** 
     * Set the simulated battery's loaded supply voltage by utilizing the total current load accumulated through {@link #addCurrentLoad}
     * calls within each existing mechanism subclass periodic. Uses {@link RoboRioSim#setVInVoltage(double)} to set the simulated battery 
     * voltage, which can then be retrieved with the {@link edu.wpi.first.wpilibj.RobotController#getBatteryVoltage()} method. This function 
     * assumes a nominal voltage of 12V and a resistance of 20 milliohms (0.020 ohms).
     */
    public static void setSimulatedBatteryLoadedVoltage() {
        double rawLoadedBatteryVoltageVolts = BatterySim.calculateDefaultBatteryLoadedVoltage(simCurrentSum.in(Amps));
        // Allow regen to reduce sag, but don't allow it to boost battery voltage beyond nominal limit
        RoboRioSim.setVInVoltage(MathUtil.clamp(rawLoadedBatteryVoltageVolts, 0, 12));
        zeroCurrents();
    }

    private static void zeroCurrents() {
        simCurrentSum = Amps.zero();
    }  
}
