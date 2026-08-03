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
import static edu.wpi.first.units.Units.Newtons;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

import frc.lib.mechanisms.Mechanism;
import lombok.AccessLevel;
import lombok.Getter;
import lombok.NoArgsConstructor;

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
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class PowerProfiler {

    @Getter(lazy = true)
    private static final PowerProfiler instance = new PowerProfiler();

    /** Record for registering a subsystem's mechanism. */
    public record MechanismRegistration(String key, Mechanism<?> mechanism) {}

    /** Record for registering a generic electrical power draw. */
    public record GenericRegistration(
            String key, DoubleSupplier currentAmpsSupplier, DoubleSupplier suppliedVoltSupplier) {}

    private record MechanicalReport(AngularVelocity mSpeed, Torque mTorque, double mEfficiency,
    AngularVelocity mechSpeed, Torque mechTorque, Force mechForce) {}

    private final List<MechanismRegistration> mechanisms = new ArrayList<>();
    private final List<GenericRegistration> generics = new ArrayList<>();

    // Per loop robot-level battery voltage
    private double batteryVoltage = 0.0;

    // Per loop robot-level current draw from battery
    private double totalCurrentAmps = 0.0;
    // Subsystem & mechanism level currents
    private final Map<String, Double> subsystemCurrents = new HashMap<>();
    // Per loop robot-level power draw from battery
    private double totalPowerWatts = 0.0;
    // Subsystem & mechanism level powers
    private final Map<String, Double> subsystemPowers = new HashMap<>();
    // Accumulated robot-level energy draw from battery since boot
    private double totalEnergyJoules = 0.0;
    // Subsystem & mechanism level energies
    private final Map<String, Double> subsystemEnergies = new HashMap<>();

    // Single motor raw rotor speed (i.e. before gear ratio)
    private double motorGroupSpeedRadPerSec = 0.0;
    private final Map<String, Double> motorSpeeds = new HashMap<>();
    // Single motor raw rotor torque (i.e. before gear ratio)
    private double singleMotorTorqueNewtonMeters = 0.0;
    private final Map<String, Double> motorTorques = new HashMap<>();
    // Single motor efficiency (i.e. raw rotor mechanical power 
    // [before gear ratio] out / battery power in)
    private double batteryToRotorEfficiency = 0.0;
    private final Map<String, Double> batteryToMotorEfficiencies = new HashMap<>();

    // Mechanism speed (after gear ratio)
    private double mechanismSpeedRadPerSec = 0.0;
    private final Map<String, Double> mechanismSpeeds = new HashMap<>();
    // Total mechanism torque (i.e. after gear ratio torque of ALL motors in the motor group)
    private double totalMechanismTorqueNewtonMeters = 0.0;
    private final Map<String, Double> totalMechanismTorques = new HashMap<>();
    // Total mechanism linear force (i.e. after gear ratio torque of ALL motors in the motor group / mechanism radius)
    private double totalMechanismForceNewtons = 0.0;
    private final Map<String, Double> totalMechanismForces = new HashMap<>();

    private boolean isInitialized = false;
    private double lastTimestamp = 0.0;
    // Battery power cut off for divide-by-zero guard
    private static final double EPSILON = 1.0;

    private static final double DEFAULT_LOOP_TIME_SECONDS = 0.02;
    private static final double MAX_LOOP_TIME_SECONDS = 0.1;

    /**
     * Register a subsystem's mechanism to the power profiler.
     * For example, "Shooter/Hood", "Shooter/Flywheel", or "Intake/Linear".
     * Not intended to register subsystems, actuators, or general power channels.
     *
     * @param key a key to log under
     * @param mechanism a Mechanism to register
     */
    public void registerMechanism(String key, Mechanism<?> mechanism) {
        mechanisms.add(new MechanismRegistration(key, mechanism));
    }

    /**
     * Register a generic power channel to the power profiler.
     * For example, "Vision/Arducam", "Vision/Jetson", or "ObjectDetection/Arducam".
     * Not intended to register subsystems, actuators, or mechanisms.
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
        batteryVoltage = Math.abs(RobotController.getBatteryVoltage());

        // Mechanisms (electrical & mechanical)
        for (var reg : mechanisms) {
            /** Loop cache */
            Mechanism<?> mechanism = reg.mechanism();
            int numMotors = mechanism.getNumberOfMotors();
            Logger.recordOutput("PowerProfiler/NumRegisteredMotors/" + reg.key(), numMotors);

            /** Battery report */ 
            // Approximation: total mechanism supply current ~ leader supply current * total motor count 
            double currentAmps = Math.abs(mechanism.getSupplyCurrent().in(Amps)) * numMotors;
            // Battery supply voltage to motor controller. Approximation: bus voltage ~ constant for all motors
            double suppliedVolts = Math.abs(mechanism.getSupplyVoltage().in(Volts));
            // Battery totalizer - add mechanism's current, power, and energy draw to the robot and subsystem-level totals
            reportElectricalUsage(reg.key(), currentAmps, suppliedVolts, loopTimeSeconds);

            /** Mechanical report */ 
            OptionalDouble Kt = mechanism.getMotorTorqueConstant();
            boolean valid = Kt.isPresent();
            double sentinel = Double.NaN;
            MechanicalReport mechanicalReport;

            // Motor
            motorGroupSpeedRadPerSec = Math.abs(mechanism.getVelocity().times(mechanism.getRotorToMechanismRatio()).in(RadiansPerSecond));
            singleMotorTorqueNewtonMeters = valid 
            ? Math.abs(mechanism.getTorqueCurrent().in(Amps) * Kt.getAsDouble()) 
            : sentinel;
            double singleMotorPower = singleMotorTorqueNewtonMeters * motorGroupSpeedRadPerSec;
            double singleMotorBatteryPower = suppliedVolts * (currentAmps / numMotors);
            batteryToRotorEfficiency = (valid && singleMotorBatteryPower > EPSILON) 
            ? singleMotorPower / singleMotorBatteryPower 
            : sentinel;

            // Mechanism
            mechanismSpeedRadPerSec = Math.abs(mechanism.getVelocity().in(RadiansPerSecond));
            totalMechanismTorqueNewtonMeters = valid 
            ? singleMotorTorqueNewtonMeters * (mechanism.getRotorToMechanismRatio() * numMotors) 
            : sentinel;
            OptionalDouble radius = mechanism.getRadius();
            totalMechanismForceNewtons = (valid && radius.isPresent()) 
            ? totalMechanismTorqueNewtonMeters / radius.getAsDouble() 
            : sentinel;

            // Mechanical totalizer
            mechanicalReport = new MechanicalReport(RadiansPerSecond.of(motorGroupSpeedRadPerSec), NewtonMeters.of
            (singleMotorTorqueNewtonMeters), batteryToRotorEfficiency, RadiansPerSecond.of(mechanismSpeedRadPerSec), NewtonMeters.of
            (totalMechanismTorqueNewtonMeters), Newtons.of(totalMechanismForceNewtons));
            reportMechanicalUsage(reg.key(), mechanicalReport);
        }

        // Generic power channels (electrical only)
        for (var reg : generics) {
            double currentAmps = Math.abs(reg.currentAmpsSupplier().getAsDouble());
            double suppliedVolts = Math.abs(reg.suppliedVoltSupplier().getAsDouble());
            reportElectricalUsage(reg.key(), currentAmps, suppliedVolts, loopTimeSeconds);
        }

        // Robot battery totals
        Logger.recordOutput("PowerProfiler/BatteryVoltageVolts", batteryVoltage);
        Logger.recordOutput("PowerProfiler/CurrentAmps", totalCurrentAmps);
        Logger.recordOutput("PowerProfiler/PowerWatts", totalPowerWatts);
        Logger.recordOutput(
                "PowerProfiler/TotalEnergyWattHours", energyToWattHours(totalEnergyJoules));

        // Subsystem / mechanism battery totals
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

        // Motor / mechanism mechanical totals
        for (var entry : motorSpeeds.entrySet()) {
            Logger.recordOutput(
                "PowerProfiler/MotorSpeedRPS/" + entry.getKey(), entry.getValue());
        }
        for (var entry : motorTorques.entrySet()) {
            Logger.recordOutput("PowerProfiler/MotorTorquesNM/" + entry.getKey(), entry.getValue());
        }
        for (var entry : batteryToMotorEfficiencies.entrySet()) {
            Logger.recordOutput("PowerProfiler/MotorEfficiencies/" + entry.getKey(), entry.getValue());
        }
        for (var entry : mechanismSpeeds.entrySet()) {
            Logger.recordOutput("PowerProfiler/MechanismSpeedsRPS/" + entry.getKey(), entry.getValue());
        }
        for (var entry : totalMechanismTorques.entrySet()) {
            Logger.recordOutput("PowerProfiler/MechanismTorquesNM/" + entry.getKey(), entry.getValue());
        }
        for (var entry : totalMechanismForces.entrySet()) {
            Logger.recordOutput("PowerProfiler/MechanismForcesN/" + entry.getKey(), entry.getValue());
        }

        // Reset loop totals (current, power, mechanical) but maintain accumulated values (energy)
        resetLoopTotals();
    }

    /** Record a mechanism/generic update and tally new resulting subsystem/mechanism battery totals */
    private void reportElectricalUsage(
            String key, double currentAmps, double suppliedVolts, double loopTimeSeconds) {
        double batteryPowerWatts = currentAmps * suppliedVolts;
        double batteryEnergyJoules = batteryPowerWatts * loopTimeSeconds;

        // New robot-level battery draw totals
        totalCurrentAmps += currentAmps;
        totalPowerWatts += batteryPowerWatts;
        totalEnergyJoules += batteryEnergyJoules;

        // New mechanism (e.g. Shooter/Hood, Shooter/Flywheel) totals
        subsystemCurrents.merge(key, currentAmps, Double::sum);
        subsystemPowers.merge(key, batteryPowerWatts, Double::sum);
        subsystemEnergies.merge(key, batteryEnergyJoules, Double::sum);

        // New subsystem totals (e.g. Shooter)
        rollUpSubsystemTotals(key, currentAmps, batteryPowerWatts, batteryEnergyJoules);
    }

    private void reportMechanicalUsage(String key, MechanicalReport mechanicalReport) {
            motorSpeeds.put(key, mechanicalReport.mSpeed().in(RotationsPerSecond));
            motorTorques.put(key, mechanicalReport.mTorque().in(NewtonMeters));
            batteryToMotorEfficiencies.put(key, mechanicalReport.mEfficiency());
            mechanismSpeeds.put(key, mechanicalReport.mechSpeed().in(RotationsPerSecond));
            totalMechanismTorques.put(key, mechanicalReport.mechTorque().in(NewtonMeters));
            totalMechanismForces.put(key, mechanicalReport.mechForce().in(Newtons));
        }

    /** Roll up the subsystem totals from the mechanism level keys. For example, this method would sum the
     * Shooter/Flywheel and Shooter/Hood mechanism-level currents, powers, and energies into the shooter 
     * subsystem key. Individual mechanism keys are updated in {@link reportElectricalUsage}. */
    private void rollUpSubsystemTotals(
            String key, double currentAmps, double powerWatts, double energyJoules) {
        String[] parts = key.split("/");
        if (parts.length < 2) return; 

        String prefix = "";
        for (int i = 0; i < parts.length - 1; i++) { 
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

    // Reset loop totals (current, power, mechanical) but maintain accumulated values (energy)
    private void resetLoopTotals() {
        totalCurrentAmps = 0.0;
        totalPowerWatts = 0.0;

        subsystemCurrents.replaceAll((k, v) -> 0.0);
        subsystemPowers.replaceAll((k, v) -> 0.0);

        motorSpeeds.replaceAll((k, v) -> 0.0);
        motorTorques.replaceAll((k, v) -> 0.0);
        batteryToMotorEfficiencies.replaceAll((k, v) -> 0.0);

        mechanismSpeeds.replaceAll((k, v) -> 0.0);
        totalMechanismTorques.replaceAll((k, v) -> 0.0);
        totalMechanismForces.replaceAll((k, v) -> 0.0);
    }

    // 1 W*h = 1 J/s * h = 1 J/s * 3600 s = 3600 J
    private static double energyToWattHours(double energyJoules) {
        return energyJoules / 3600.0;
    }
}
