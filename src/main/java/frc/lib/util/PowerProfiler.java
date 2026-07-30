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
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.wpilibj.Timer;

import frc.lib.mechanisms.Mechanism;

import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

/**
 * A power profiling utility used to estimate time-dependent current/power/energy draw from the
 * robot's battery as a function of subsystem and mechanism.
 */
public class PowerProfiler {

    public record MechanismRegistration(String key, Mechanism<?> mechanism) {}

    public record GenericRegistration(
            String key, DoubleSupplier currentAmpsSupplier, DoubleSupplier suppliedVoltSupplier) {}

    private record MechanicalReport(double bEfficiency, AngularVelocity mSpeed, Torque mTorque, double mEfficiency,
    AngularVelocity mechSpeed, Torque mechTorque, double mechEfficiency) {}

    private final List<MechanismRegistration> mechanisms = new ArrayList<>();
    private final List<GenericRegistration> generics = new ArrayList<>();

    // Per loop current draw from battery
    private double totalCurrentAmps = 0.0;
    private final Map<String, Double> subsystemCurrents = new HashMap<>();
    // Per loop power draw from battery
    private double totalPowerWatts = 0.0;
    private final Map<String, Double> subsystemPowers = new HashMap<>();
    // Accumulated energy draw from battery since boot
    private double totalEnergyJoules = 0.0;
    private final Map<String, Double> subsystemEnergies = new HashMap<>();

    // Per loop bus -> motor efficiency
    private double busEfficiency = 0.0;

    // Per loop motor speed
    private double motorSpeedRadPerSec = 0.0;
    private Map<String, Double> motorSpeeds = new HashMap<>();
    // Per loop motor torque
    private double motorTorqueNewtonMeters = 0.0;
    private Map<String, Double> motorTorques = new HashMap<>();
    // Per loop motor efficiency
    private double motorEfficiency = 0.0;
    private Map<String, Double> motorEfficiencies = new HashMap<>();

    // Per loop mechanism speed
    private double mechanismSpeedRadPerSec = 0.0;
    private Map<String, Double> mechanismSpeeds = new HashMap<>();
    // Per loop mechanism torque
    private double mechanismTorqueNewtonMeters = 0.0;
    private Map<String, Double> mechanismTorques = new HashMap<>();
    // Per loop mechanism efficiency
    private double mechanismEfficiency = 0.0;
    private Map<String, Double> mechanismEfficiencies = new HashMap<>();

    private boolean isInitialized = false;
    private double lastTimestamp = 0.0;

    private static final double DEFAULT_LOOP_TIME_SECONDS = 0.02;
    private static final double MAX_LOOP_TIME_SECONDS = 0.1;

    /**
     * Register a mechanism to the power profiler (e.g. rotary, linear)
     *
     * @param key a key to log under
     * @param mechanism a Mechanism to register
     */
    public void registerMechanism(String key, Mechanism<?> mechanism) {
        mechanisms.add(new MechanismRegistration(key, mechanism));
    }

    /**
     * Register a generic power channel to the power profiler (e.g. drive module, arducam)
     *
     * @param key a key to log under
     * @param currentAmpsSupplier supply current supplier in Amps
     * @param suppliedVoltSupplier supply voltage supplier in Volts
     */
    public void registerGeneric(
            String key, DoubleSupplier currentAmpsSupplier, DoubleSupplier suppliedVoltsSupplier) {
        generics.add(new GenericRegistration(key, currentAmpsSupplier, suppliedVoltsSupplier));
    }

    /**
     * Loops through each registered mechanism and generic power channel, retrieves its present
     * applied voltage and supply current (including the draw of any followers), adds the {current,
     * power, energy} profile results to the accumulators, logs the full profile, and attributes it
     * by subsystem/mechanism/generic. Resets the per-loop values (current, power) every scan while
     * maintaining energy tracking since boot.
     */
    public void periodicAfterScheduler() {
        double loopTimeSeconds = getLoopTime();
        for (var reg : mechanisms) {
            /** Loop cache */
            Mechanism<?> mechanism = reg.mechanism();
            int numMotors = mechanism.getNumberOfMotors();
            Logger.recordOutput("PowerProfiler/NumRegisteredMotors/" + reg.key(), numMotors);

            /** Battery report */
            // Approximation: total mechanism supply current ~ leader supply current * total motor
            // count. Bus voltage ~ constant for all motors.
            double currentAmps = Math.abs(mechanism.getSupplyCurrent().in(Amps)) * numMotors;
            // Power supplied to motor group from battery = supply voltage * current draw
            double suppliedVolts = Math.abs(mechanism.getSupplyVoltage().in(Volts));
            // Power supplied to mechanism from motor group = applied voltage * current draw
            double appliedVolts = Math.abs(mechanism.getAppliedVoltage().in(Volts));
            // Battery totalizer
            reportElectricalUsage(reg.key(), currentAmps, suppliedVolts, loopTimeSeconds);

            /** Mechanical report */
            OptionalDouble Kt = mechanism.getMotorTorqueConstant();
            boolean valid = Kt.isPresent();
            double sentinel = Double.NaN;
            MechanicalReport mechanicalReport;
            // Bus
            busEfficiency = appliedVolts / suppliedVolts;
            // Motor
            motorSpeedRadPerSec = Math.abs(mechanism.getVelocity().times(mechanism.getRotorToMechanismRatio()).in(RadiansPerSecond));
            motorTorqueNewtonMeters = valid ? Math.abs(mechanism.getTorqueCurrent().in(Amps) * Kt.getAsDouble()) : sentinel;
            double motorPower = motorTorqueNewtonMeters * motorSpeedRadPerSec;
            motorEfficiency = valid ? motorPower / ((currentAmps / numMotors) * appliedVolts) : sentinel;
            // Mechanism
            mechanismSpeedRadPerSec = Math.abs(mechanism.getVelocity().in(RadiansPerSecond));
            mechanismTorqueNewtonMeters = valid ? motorTorqueNewtonMeters * (mechanism.getRotorToMechanismRatio() * numMotors) : sentinel;
            double mechanismPower = mechanismTorqueNewtonMeters * mechanismSpeedRadPerSec;
            mechanismEfficiency = valid ? mechanismPower / (motorPower * numMotors) : sentinel;
            // Mechanical totalizer
            mechanicalReport = new MechanicalReport(busEfficiency, RadiansPerSecond.of(motorSpeedRadPerSec), NewtonMeters.of
            (motorTorqueNewtonMeters), motorEfficiency, RadiansPerSecond.of(mechanismSpeedRadPerSec), NewtonMeters.of
            (mechanismTorqueNewtonMeters), mechanismEfficiency);
            reportMechanicalUsage(reg.key(), mechanicalReport);
        }

        for (var reg : generics) {
            double currentAmps = Math.abs(reg.currentAmpsSupplier().getAsDouble());
            double suppliedVolts = Math.abs(reg.suppliedVoltSupplier().getAsDouble());
            reportElectricalUsage(reg.key(), currentAmps, suppliedVolts, loopTimeSeconds);
        }

        Logger.recordOutput("PowerProfiler/CurrentAmps", totalCurrentAmps);
        Logger.recordOutput("PowerProfiler/PowerWatts", totalPowerWatts);
        Logger.recordOutput(
                "PowerProfiler/TotalEnergyWattHours", energyToWattHours(totalEnergyJoules));

        for (var entry : subsystemCurrents.entrySet()) {
            Logger.recordOutput("PowerProfiler/CurrentAmps/" + entry.getKey(), entry.getValue());
        }
        for (var entry : subsystemPowers.entrySet()) {
            Logger.recordOutput("PowerProfiler/PowerWatts/" + entry.getKey(), entry.getValue());
        }
        for (var entry : subsystemEnergies.entrySet()) {
            Logger.recordOutput(
                    "PowerProfiler/EnergyWattHours/" + entry.getKey(),
                    energyToWattHours(entry.getValue()));
        }
        resetLoopTotals();
    }

    // Record a mechanism/generic update and tally new resulting subsystem/mechanism battery totals
    private void reportElectricalUsage(
            String key, double currentAmps, double suppliedVolts, double loopTimeSeconds) {
        double batteryPowerWatts = currentAmps * suppliedVolts;
        double batteryEnergyJoules = batteryPowerWatts * loopTimeSeconds;

        // New battery draw totals
        totalCurrentAmps += currentAmps;
        totalPowerWatts += batteryPowerWatts;
        totalEnergyJoules += batteryEnergyJoules;

        // New mechanism (e.g. PowerProfiler/shooter/hood, PowerProfiler/shooter/flywheel) totals
        subsystemCurrents.merge(key, currentAmps, Double::sum);
        subsystemPowers.merge(key, batteryPowerWatts, Double::sum);
        subsystemEnergies.merge(key, batteryEnergyJoules, Double::sum);

        // New robot/subsystem totals (e.g. PowerProfiler/shooter)
        rollUpSubsystemTotals(key, currentAmps, batteryPowerWatts, batteryEnergyJoules);
    }

    private void reportMechanicalUsage(String key, MechanicalReport mechanicalReport) {
            String suffix = "/Mechanical";
            motorSpeeds.put(key + suffix, mechanicalReport.mSpeed().in(RotationsPerSecond));
            motorTorques.put(key + suffix, mechanicalReport.mTorque().in(NewtonMeters));
            motorEfficiencies.put(key + suffix, mechanicalReport.mEfficiency());
            mechanismSpeeds.put(key + suffix, mechanicalReport.mechSpeed().in(RotationsPerSecond));
            mechanismTorques.put(key + suffix, mechanicalReport.mechTorque().in(NewtonMeters));
            mechanismEfficiencies.put(key + suffix, mechanicalReport.mechEfficiency());
        }

    // Roll up the subsystem totals from the mechanism level
    private void rollUpSubsystemTotals(
            String key, double currentAmps, double powerWatts, double energyJoules) {
        String[] parts = key.split("/");
        if (parts.length < 2) return; // might need to change to properly register generics

        String prefix = "";
        for (int i = 0; i < parts.length - 1; i++) { // does this want to be parts.length - 2 because we're double counting mechanisms?
            prefix = prefix.isEmpty() ? parts[i] : prefix.concat("/").concat(parts[i]);
            subsystemCurrents.merge(prefix, currentAmps, Double::sum);
            subsystemPowers.merge(prefix, powerWatts, Double::sum);
            subsystemEnergies.merge(prefix, energyJoules, Double::sum);
        }
    }

    // RIO loop time in seconds
    private double getLoopTime() {
        double now = Timer.getTimestamp();

        if (!isInitialized) {
            lastTimestamp = now;
            isInitialized = true;
            return DEFAULT_LOOP_TIME_SECONDS;
        }

        if (Logger.hasReplaySource()) {
            lastTimestamp = now;
            return DEFAULT_LOOP_TIME_SECONDS;
        }

        double dt = now - lastTimestamp;
        lastTimestamp = now;

        if (dt <= 0.0) {
            return DEFAULT_LOOP_TIME_SECONDS;
        }

        return Math.min(dt, MAX_LOOP_TIME_SECONDS);
    }

    // Reset loop totals but maintain accumulated values
    private void resetLoopTotals() {
        totalCurrentAmps = 0.0;
        totalPowerWatts = 0.0;
        subsystemCurrents.replaceAll((k, v) -> 0.0);
        subsystemPowers.replaceAll((k, v) -> 0.0);
    }

    // 1 W*h = 1 J/s * h = 1 J/s * 3600 s = 3600 J
    private static double energyToWattHours(double energyJoules) {
        return energyJoules / 3600.0;
    }
}
