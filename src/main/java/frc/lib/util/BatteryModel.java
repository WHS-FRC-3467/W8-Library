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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;

/** A utility class simulating a battery. */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public final class BatteryModel {
    /* Battery nominal voltage to simulate in Volts.
     * This is the voltage of a fully charged battery with no load.
     * Set this field manually as desired to simulate various electrical performances. 
     */
    private static final double batteryNominalVoltageVolts = 12.0;
    /* Battery internal resistance to simulate in Ohms.
     * This is the internal resistance of the battery between the battery terminals and the load.
     * Set this field manually as desired to simulate various electrical performances. 
     */
    private static final double batteryInternalResistanceOhms = 0.011; 

    /** 
     * The accumulated signed supply-current load on the simulated battery.
     * This field is set programatically by the {@link #addCurrentLoad(Current)} method.
     */ 
    private static double simCurrentSumAmps = 0.0;

    // Battery voltage LPF parameters
    private static final double LOOP_TIME_SECONDS = 0.02; 
    private static final double BATTERY_VOLTAGE_TIME_CONSTANT_SECONDS = 0.10;
    
    /**
     * Adds a mechanism's signed battery/supply current to the accumulator.
     *
     * <p>Positive current is treated as battery draw and increases battery sag. Negative current is treated 
     * as regeneration to the bus and reduces sag. Callers should pass signed supply/battery current, not 
     * torque/stator current.
     *
     * @param currentAmps signed supply current in Amps
     */
    public static void addCurrentLoad(double currentAmps) {
        simCurrentSumAmps += currentAmps;
    }

    /** 
     * Set the simulated battery's loaded supply voltage by utilizing the total current load accumulated through {@link #addCurrentLoad}
     * calls within each existing mechanism subclass periodic. Uses {@link RoboRioSim#setVInVoltage(double)} to set the simulated battery 
     * voltage, which can then be retrieved with the {@link edu.wpi.first.wpilibj.RobotController#getBatteryVoltage()} method. This function 
     * assumes a nominal voltage of 12V and a resistance {@link #batteryInternalResistanceOhms}.
     */
    public static void setSimulatedBatteryLoadedVoltage() {
        // Calculate the target loaded battery voltage based on the nominal voltage, internal resistance, and total current draw
        double targetLoadedBatteryVoltageVolts = calculateDefaultBatteryLoadedVoltage(batteryNominalVoltageVolts, batteryInternalResistanceOhms, 
            simCurrentSumAmps);
        // LPF the battery voltage to avoid sudden jumps in voltage when the current draw changes
        double alpha = LOOP_TIME_SECONDS / (LOOP_TIME_SECONDS + BATTERY_VOLTAGE_TIME_CONSTANT_SECONDS);
        double previousVoltageVolts = RobotController.getBatteryVoltage();
        double filteredVoltageVolts = previousVoltageVolts + alpha * (targetLoadedBatteryVoltageVolts - previousVoltageVolts);
        // Don't allow the battery voltage to go below 0 V or above 12 V
        RoboRioSim.setVInVoltage(MathUtil.clamp(filteredVoltageVolts, 0.0, batteryNominalVoltageVolts));
        zeroCurrents();
    }

    /**
     * Calculates the loaded battery voltage based on the nominal voltage, internal resistance, and current draw.
     * 
     * @param nominalVoltageVolts Nominal voltage of the battery in volts (e.g., 12V for a standard FRC battery)
     * @param resistanceOhms Internal resistance of the battery in ohms (e.g., 0.011 Ohms for a typical new FRC battery,
     * up to 0.025 Ohms for an aged battery)
     * @param currentAmps Total current draw from the battery in amps (positive for draw, negative for regen)
     * @return The loaded battery voltage in volts after accounting for voltage sag due to current draw
     */
    private static double calculateDefaultBatteryLoadedVoltage(double nominalVoltageVolts, double resistanceOhms, double currentAmps) {
        double voltageSagVolts = currentAmps * resistanceOhms;
        return nominalVoltageVolts - voltageSagVolts;
    }

    private static void zeroCurrents() {
        simCurrentSumAmps = 0.0;
    }  
}
