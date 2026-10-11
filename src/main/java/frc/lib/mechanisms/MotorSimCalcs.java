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
package frc.lib.mechanisms;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;

/** A utility class providing calculation support for motor sims. */ 
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public final class MotorSimCalcs {

    /**
    * Calculates the supply current draw from the battery based on the applied voltage, bus voltage, and torque current.
    * Can be positive or negative depending on whether the motor is drawing current from the battery or regenerating back to it.
    * 
    * @param batteryVoltageVolts Battery (bus) voltage magnitude in Volts. 
    * @param appliedVoltageVolts Motor applied voltage in Volts. Positive or negative.
    * @param torqueCurrentAmps Torque (stator) current in Amps. Positive or negative. 
    * @return Supply current draw in Amps
    */
    public static double getSupplyCurrentAmps(double batteryVoltageVolts, double appliedVoltageVolts, double torqueCurrentAmps) {
        batteryVoltageVolts = Math.abs(batteryVoltageVolts);
        if (batteryVoltageVolts < 1e-6) {
            return 0.0;
        }
        double motorPowerWatts = torqueCurrentAmps * appliedVoltageVolts;
        double supplyCurrentAmps = motorPowerWatts / batteryVoltageVolts;

        return supplyCurrentAmps;
    }

    /**
     * Attempt to clamp the requested voltage to the torque current limit of the motor, if the presently available battery 
     * voltage allows. Clamps both positive and negative currents. 
     * 
     * @param requestedVolts Applied volts requested by the controller (with proper sign to indicate polarity)
     * @param mechanismVelocityRadPerSec Current mechanism/output-side angular velocity in rad/s, using the same sign 
     * convention as the simulated motor input voltage
     * @param gearing Gear ratio from motor to mechanism/output
     * @param motor DCMotor model of the motor being simulated (bare, unreduced, qty 1)
     * @param currentLimitAmps Torque / stator current limit magnitude for the motor in Amps. Note that this is
     * different from a supply current limit, which would require a different method. 
     * @param batteryVoltageVolts The current available battery voltage magnitude, assumed usable bidirectionally
     * @return An updated applied voltage request, clamped to the current limit if necessary
     */
    public static double clampVoltageToTorqueCurrentLimit(double requestedVolts, double mechanismVelocityRadPerSec, double gearing,
    DCMotor motor, double currentLimitAmps, double batteryVoltageVolts) {
            /*
            * Values from DCMotorSim/DCMotor are internally self-consistent: motor applied voltage,
            * simulated output angular position/velocity, simulated output torque,
            * and derived motor torque current all use the same simulated positive
            * direction. This does not imply anything about real-world motor/controller
            * inversion; it is only a local convention inside the sim.
            */
            double motorVelocityRadPerSec = mechanismVelocityRadPerSec * gearing;
            double currentLimitMagnitudeAmps = Math.abs(currentLimitAmps);
            double batteryVoltageMagnitudeVolts = Math.abs(batteryVoltageVolts);

            /* 
             * Simple current model: I = (V_applied - w / Kv) / R 
             * Therefore, for | I | <= I_magnitude_limit,
             * | V_applied - w / Kv | <= I_magnitude_limit * R
             * -I_magnitude_limit * R <= V_applied - w / Kv <= I_magnitude_limit * R
             * w / Kv - I_magnitude_limit * R <= V_applied <= I_magnitude_limit * R + w / Kv
             */
            double zeroCurrentVolts = motorVelocityRadPerSec / motor.KvRadPerSecPerVolt;
            double voltageMarginVolts = currentLimitMagnitudeAmps * motor.rOhms;
            double currentLimitedMinVolts = zeroCurrentVolts - voltageMarginVolts;
            double currentLimitedMaxVolts = zeroCurrentVolts + voltageMarginVolts;
            // But also: -V_battery <= V_applied <= V_battery
            double batteryLimitedMinVolts = -batteryVoltageMagnitudeVolts;
            double batteryLimitedMaxVolts = batteryVoltageMagnitudeVolts;
            /*
             * Therefore, max(w / Kv - I_magnitude_limit * R, - V_battery) <= V_applied 
             * <= min(I_magnitude_limit * R + w / Kv, V_battery) is the voltage range that 
             * satisfies both the current limit and the presently available battery voltage.
             */
            double feasibleMinVolts = Math.max(currentLimitedMinVolts, batteryLimitedMinVolts);
            double feasibleMaxVolts = Math.min(currentLimitedMaxVolts, batteryLimitedMaxVolts);

            // If the intervals overlap, there is a voltage within the presently available
            // battery voltage range that can satisfy the current limit. Clamp the requested
            // voltage into that feasible interval.
            if (feasibleMinVolts <= feasibleMaxVolts) {
                return MathUtil.clamp(requestedVolts, feasibleMinVolts, feasibleMaxVolts);
            }
            // If the intervals do not overlap, no available battery voltage can satisfy the
            // current limit. Apply the available voltage closest to the zero-current voltage,
            // which minimizes current magnitude. Due to voltage saturation, the current limit
            // will be exceeded.
            return MathUtil.clamp(zeroCurrentVolts, batteryLimitedMinVolts, batteryLimitedMaxVolts);
        }

    /** Return the DCMotorSim's current torque current in Amps.
     * 
     * @param sim DCMotor sim representing the gearbox producing the torque current
     * @param motorKtNmPerAmp Motor torque constant in Nm/Amp for the motor being simulated
     * @return The torque (stator) current in Amps, which can be positive or negative, following the 
     * direction of the torque.
     */
    public static double getTorqueCurrentAmps(DCMotorSim sim, double motorKtNmPerAmp) {
        // Gearing = motor-to-mechanism speed ratio (magnitude)
        return sim.getTorqueNewtonMeters() / (Math.abs(sim.getGearing() * motorKtNmPerAmp));
    }
}
