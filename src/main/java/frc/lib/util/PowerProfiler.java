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
import static edu.wpi.first.units.Units.Watts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Power;
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
 * A power profiling utility used to estimate the robot's time-dependent current/power/energy 
 * drain and output as a function of subsystem, mechanism, motor group, and generic power channel.
 * 
 * Core assumptions:
 * <ol>
 *  <li>Follower motors draw roughly the same current as the leader.
 *  <li>All motors in a mechanism have the same motor model / Kt.
 *  <li>All motors contribute torque in the same mechanism direction.
 *  <li>Rotor-to-mechanism ratio and Kt are configured correctly. Rotor-to-mechanism
 *      ratio should be included in the TalonFX configuration and Kt should be included
 *      through a motor model in the overloaded TalonFX constructor as outlined in the 
 *      corresponding Javadoc. 
 *  <li>Supply voltage is approximately common across controllers.
 *  <li>Mechanism torque is ideal, with no gearbox/belt/chain losses.
 *  <li>Bus regen is not accounted for. This profiler only tallies power drains (i.e. positive 
 *      current draws) -- negative current draw is ignored and therefore battery regen is neglected.
 * </ol>
 * 
 */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class PowerProfiler {

    @Getter(lazy = true)
    private static final PowerProfiler instance = new PowerProfiler();

    /* Record for registering a subsystem's mechanism. */
    public record MechanismRegistration(String key, Mechanism<?> mechanism) {}

    /* Record for registering a generic electrical power draw. */
    public record GenericRegistration(
            String key, DoubleSupplier currentAmpsSupplier, DoubleSupplier suppliedVoltSupplier) {}

    private record MechanicalReport(AngularVelocity mSpeed, Torque mTorque,
    AngularVelocity mechSpeed, Torque mechTorque, Power mechPower, double estEta) {}

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
    private double motorSpeedRadPerSec = 0.0;
    private final Map<String, Double> motorSpeeds = new HashMap<>();
    // Single motor raw rotor torque (i.e. before gear ratio)
    private double singleMotorTorqueNM = 0.0;
    private final Map<String, Double> motorTorques = new HashMap<>();

    // Mechanism velocity (i.e. after gear ratio)
    private double mechanismVelocityRadPerSec = 0.0;
    private final Map<String, Double> mechanismVelocities = new HashMap<>();
    // Total mechanism torque magnitude (i.e. after gear ratio torque magnitude of ALL motors in the motor group)
    private double totalMechanismTorqueMagNM = 0.0;
    private final Map<String, Double> totalMechanismTorqueMags = new HashMap<>();
    /*
    * Total estimated mechanism mechanical power magnitude:
    *
    *   |P_mech| ≈ |T_mech| * |ω_mech|
    *
    * This profiler currently has no insight into whether sign inversion exists between
    * the motor and mechanism coordinate systems, so this is reported as a magnitude. The signed quantity
    * indicates the directionality of the power transfer -- positive is motoring, negative is braking. 
    * Use command/state/log context to determine whether the mechanism was motoring or braking at a given timestamp.
    *
    * Motoring refers to increasing the energy of the output load (e.g. accelerating a flywheel).
    * Braking refers to decreasing the energy of the output load (e.g. decelerating a flywheel).
    */
    private double totalMechanismMechPowerMagWatts = 0.0;
    private final Map<String, Double> totalMechanismPowerMags = new HashMap<>();
    /*
    * Empirical diagnostic ratio, not a guaranteed true efficiency measurement.
    *
    * This is only meaningful when external context confirms the mechanism is
    * motoring. It should not be trusted during braking, backdriving,
    * backlash, impacts, severe slip, stalls/hard-stops, or other cases where
    * the estimated mechanism power magnitude is not useful output power.
    *
    * Values above 1.0 indicate invalid assumptions or an operating condition
    * outside this model.
    */
    private double estBatteryToMechanismEfficiency = 0.0;
    private final Map<String, Double> estBatteryToMechanismEfficiencies = new HashMap<>();

    private boolean isInitialized = false;
    private double lastTimestamp = 0.0;

    private static final double DEFAULT_LOOP_TIME_SECONDS = 0.02;
    private static final double MAX_LOOP_TIME_SECONDS = 0.1;
    private static final double EPSILON = 1.0;

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
     * @param currentAmpsSupplier supply current supplier in Amps (positive for draw, 
     * negative for regen)
     * @param suppliedVoltsSupplier supply voltage supplier in Volts
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
            // Cache
            Mechanism<?> mechanism = reg.mechanism();
            int numMotors = mechanism.getNumberOfMotors();
            Logger.recordOutput("PowerProfiler/NumRegisteredMotors/" + reg.key(), numMotors);

            /* Battery report */ 
            // Approximation: total mechanism supply (battery) current ~ leader supply (battery) current * total motor count 
            // Treat negatively-signed supply current as bus return and ignore it for drain attribution
            double drawCurrentAmps = Math.max(0.0, mechanism.getSupplyCurrent().in(Amps) * numMotors);
            // Supply (battery) voltage to motor controller. Approximation: bus voltage ~ constant for all motors in motor group
            double suppliedVolts = Math.abs(mechanism.getSupplyVoltage().in(Volts));
            // Battery totalizer - add mechanism's current, power, and energy draw to the robot and subsystem-level totals
            reportElectricalUsage(reg.key(), drawCurrentAmps, suppliedVolts, loopTimeSeconds);

            /* Mechanical report */ 
            OptionalDouble Kt = mechanism.getMotorTorqueConstant();
            boolean valid = Kt.isPresent();
            double sentinel = Double.NaN;

            // Motor
            // No insight into gearing/belt inversion, so don't assume sign
            motorSpeedRadPerSec = Math.abs(mechanism.getVelocity().times(mechanism.getRotorToMechanismRatio()).in(RadiansPerSecond));
            singleMotorTorqueNM = 
                valid 
                    ? mechanism.getTorqueCurrent().in(Amps) * Kt.getAsDouble() 
                    : sentinel;

            // Mechanism
            mechanismVelocityRadPerSec = mechanism.getVelocity().in(RadiansPerSecond);
            // No insight into gearing/belt inversion, so don't assume sign
            totalMechanismTorqueMagNM = 
                valid 
                    ? Math.abs(singleMotorTorqueNM * mechanism.getRotorToMechanismRatio() * numMotors)
                    : sentinel;
            totalMechanismMechPowerMagWatts = valid ? 
                totalMechanismTorqueMagNM * Math.abs(mechanismVelocityRadPerSec)
                : sentinel;
            double batteryPowerWatts = drawCurrentAmps * suppliedVolts;
            /*
             * This efficiency is only meaningful when the mechanism is known to be normally motoring (defined above).
             * When the motor is acting as a generator (i.e. actual mechanism power < 0), this definition of efficiency is invalid
             * and should be ignored.
             */
            estBatteryToMechanismEfficiency = valid && batteryPowerWatts > EPSILON ? 
                totalMechanismMechPowerMagWatts / batteryPowerWatts : sentinel;

            // Mechanical totalizer
            MechanicalReport mechanicalReport = new MechanicalReport(RadiansPerSecond.of(motorSpeedRadPerSec), NewtonMeters.of
            (singleMotorTorqueNM), RadiansPerSecond.of(mechanismVelocityRadPerSec), NewtonMeters.of
            (totalMechanismTorqueMagNM), Watts.of(totalMechanismMechPowerMagWatts), estBatteryToMechanismEfficiency);
            reportMechanicalUsage(reg.key(), mechanicalReport);
        }

        // Generic power channels (electrical only)
        for (var reg : generics) {
            // Treat negative signed supply current as bus return and ignore it for drain attribution
            double drawCurrentAmps = Math.max(0.0, reg.currentAmpsSupplier().getAsDouble());
            double suppliedVolts = Math.abs(reg.suppliedVoltSupplier().getAsDouble());
            reportElectricalUsage(reg.key(), drawCurrentAmps, suppliedVolts, loopTimeSeconds);
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
            Logger.recordOutput("PowerProfiler/MotorTorqueNM/" + entry.getKey(), entry.getValue());
        }
        for (var entry : mechanismVelocities.entrySet()) {
            Logger.recordOutput("PowerProfiler/MechanismVelocityRPS/" + entry.getKey(), entry.getValue());
        }
        for (var entry : totalMechanismTorqueMags.entrySet()) {
            Logger.recordOutput("PowerProfiler/MechanismTorqueMagnitudeNM/" + entry.getKey(), entry.getValue());
        }
        for (var entry : totalMechanismPowerMags.entrySet()) {
            Logger.recordOutput("PowerProfiler/MechanismPowerMagnitudeWatts/" + entry.getKey(), entry.getValue());
        }
        for (var entry : estBatteryToMechanismEfficiencies.entrySet()) {
            Logger.recordOutput("PowerProfiler/EstBatteryToMechanismEfficiency/" + entry.getKey(), entry.getValue());
        }

        // Reset loop totals (current, power, mechanical) but maintain accumulated values (energy)
        resetLoopTotals();
    }

    /** Record a mechanism/generic update and tally new resulting subsystem/mechanism battery totals */
    private void reportElectricalUsage(
            String key, double drawCurrentAmps, double suppliedVolts, double loopTimeSeconds) {
        double batteryPowerWatts = drawCurrentAmps * suppliedVolts;
        double batteryEnergyJoules = batteryPowerWatts * loopTimeSeconds;

        // New robot-level battery draw totals
        totalCurrentAmps += drawCurrentAmps;
        totalPowerWatts += batteryPowerWatts;
        totalEnergyJoules += batteryEnergyJoules;

        // New mechanism (e.g. Shooter/Hood, Shooter/Flywheel) totals
        subsystemCurrents.merge(key, drawCurrentAmps, Double::sum);
        subsystemPowers.merge(key, batteryPowerWatts, Double::sum);
        subsystemEnergies.merge(key, batteryEnergyJoules, Double::sum);

        // New subsystem totals (e.g. Shooter)
        rollUpSubsystemTotals(key, drawCurrentAmps, batteryPowerWatts, batteryEnergyJoules);
    }

    /** 
     * Record a mechanism update and tally new mechanism/motor totals -- rolling up keys 
     * is unnecessary because mechanical subsystem values are not required.
     */
    private void reportMechanicalUsage(String key, MechanicalReport mechanicalReport) {
            motorSpeeds.put(key, mechanicalReport.mSpeed().in(RotationsPerSecond));
            motorTorques.put(key, mechanicalReport.mTorque().in(NewtonMeters));
            mechanismVelocities.put(key, mechanicalReport.mechSpeed().in(RotationsPerSecond));
            totalMechanismTorqueMags.put(key, mechanicalReport.mechTorque().in(NewtonMeters));
            totalMechanismPowerMags.put(key, mechanicalReport.mechPower().in(Watts));
            estBatteryToMechanismEfficiencies.put(key, mechanicalReport.estEta());
        }

    /** 
     * Roll up the subsystem totals from the mechanism level keys. For example, this method would sum the
     * Shooter/Flywheel and Shooter/Hood mechanism-level currents, powers, and energies into the shooter 
     * subsystem key. Individual mechanism keys are updated in {@link reportElectricalUsage}. 
     */
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

        mechanismVelocities.replaceAll((k, v) -> 0.0);
        totalMechanismTorqueMags.replaceAll((k, v) -> 0.0);
        totalMechanismPowerMags.replaceAll((k, v) -> 0.0);
        estBatteryToMechanismEfficiencies.replaceAll((k, v) -> 0.0);
    }

    // 1 W*h = 1 J/s * h = 1 J/s * 3600 s = 3600 J
    private static double energyToWattHours(double energyJoules) {
        return energyJoules / 3600.0;
    }
}
