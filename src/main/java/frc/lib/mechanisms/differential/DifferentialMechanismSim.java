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

package frc.lib.mechanisms.differential;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

import frc.lib.io.absoluteencoder.AbsoluteEncoderIOSim;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.io.motor.MotorIOSim;
import frc.lib.mechanisms.differential.DifferentialMechanism.DiffMechCharacteristics;

import java.util.Optional;

/**
 * A simulated implementation of {@link DifferentialMechanism} that models each motor gearbox
 * independently using WPILib's {@link FlywheelSim}.
 *
 * <p>The average and differential positions are computed from the two simulated motors:
 *
 * <ul>
 *   <li><b>Average position</b> = (leaderPosition + followerPosition) / 2
 *   <li><b>Differential position</b> = (leaderPosition − followerPosition) / 2
 * </ul>
 *
 * <p>This matches CTRE's convention and lets teams develop/test differential control logic without
 * physical hardware.
 *
 * @see DifferentialMechanismReal for the real-hardware counterpart
 */
public class DifferentialMechanismSim
        extends DifferentialMechanism<MotorIOSim, MotorIOSim, AbsoluteEncoderIOSim> {

    /** WPILib flywheel simulation for the leader-side gearbox. */
    private final FlywheelSim leaderSim;

    /** WPILib flywheel simulation for the follower-side gearbox. */
    private final FlywheelSim followerSim;

    /** Timestamp of the previous periodic() call; used to compute deltaTime. */
    private Time lastTime = RobotController.getMeasureTime();

    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    /**
     * Constructs a simulated differential mechanism.
     *
     * <p>Each gearbox is modeled as an independent flywheel. The inertia you provide should
     * represent one gearbox's effective load. If the two gearboxes are asymmetric, create separate
     * instances and extend this class.
     *
     * @param name logical name used for logging and visualization
     * @param characteristics physical characteristics (axes, gearing, starting positions)
     * @param leaderIo simulated motor IO for the leader gearbox
     * @param followerIo simulated motor IO for the follower gearbox
     * @param motor DC motor model (e.g., {@code DCMotor.getKrakenX60(1)})
     * @param momentOfInertia moment of inertia at the mechanism output (per gearbox side)
     * @param absoluteEncoder optional simulated absolute encoder on the differential axis
     */
    public DifferentialMechanismSim(
            String name,
            DiffMechCharacteristics characteristics,
            MotorIOSim leaderIo,
            MotorIOSim followerIo,
            DCMotor motor,
            MomentOfInertia momentOfInertia,
            Optional<AbsoluteEncoderIOSim> absoluteEncoder) {
        super(name, characteristics, leaderIo, followerIo, absoluteEncoder, name + "/Encoder");

        if (momentOfInertia.isEquivalent(KilogramSquareMeters.zero())) {
            throw new IllegalArgumentException("momentOfInertia must be greater than zero!");
        }

        double jKgM2 = momentOfInertia.in(KilogramSquareMeters);
        double gearing = leaderIo.getRotorToSensorRatio() * leaderIo.getSensorToMechanismRatio();

        leaderSim =
                new FlywheelSim(LinearSystemId.createFlywheelSystem(motor, jKgM2, gearing), motor);

        followerSim =
                new FlywheelSim(LinearSystemId.createFlywheelSystem(motor, jKgM2, gearing), motor);
    }

    // -----------------------------------------------------------------------
    // Periodic — advance both simulations and compute combined axis states
    // -----------------------------------------------------------------------

    @Override
    public void periodic() {
        Time currentTime = RobotController.getMeasureTime();
        double dt = currentTime.minus(lastTime).in(Seconds);
        lastTime = currentTime;

        // --- Step each gearbox simulation ---
        // Input voltage comes from the most recent applied voltage reported by the MotorIO.
        leaderSim.setInputVoltage(inputs.appliedVoltage.in(Volts));
        followerSim.setInputVoltage(followerInputs.appliedVoltage.in(Volts));

        leaderSim.update(dt);
        followerSim.update(dt);

        // --- Update simulated battery load ---
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(
                        leaderSim.getCurrentDrawAmps() + followerSim.getCurrentDrawAmps()));

        // --- Propagate new state back into each MotorIOSim ---
        // Angular displacement: θ = ω₀t + ½αt²
        Angle leaderPositionChange =
                Radians.of(
                        leaderSim.getAngularVelocityRadPerSec() * dt
                                + 0.5 * leaderSim.getAngularAccelerationRadPerSecSq() * dt * dt);
        io.setPosition(inputs.position.plus(leaderPositionChange));
        io.setRotorVelocity(leaderSim.getAngularVelocity());
        io.setRotorAcceleration(leaderSim.getAngularAcceleration());

        Angle followerPositionChange =
                Radians.of(
                        followerSim.getAngularVelocityRadPerSec() * dt
                                + 0.5 * followerSim.getAngularAccelerationRadPerSecSq() * dt * dt);
        followerIo.setPosition(followerInputs.position.plus(followerPositionChange));
        followerIo.setRotorVelocity(followerSim.getAngularVelocity());
        followerIo.setRotorAcceleration(followerSim.getAngularAcceleration());

        // --- Delegate to the base class to log all inputs ---
        super.periodic();

        // --- Update visualizer color based on mechanism state ---
        // Green = differential axis is near zero (aligned), Yellow = in motion
        Angle diffPos = diffMechInputs.differentialPosition;
        boolean aligned = diffPos.isNear(Rotations.zero(), Rotations.of(0.05));
        visualizer.setDifferentialAngle(diffPos);
        visualizer.setAverageAngle(diffMechInputs.averagePosition);
    }

    // -----------------------------------------------------------------------
    // updateDifferentialInputs — derived from the two simulated motor positions
    // -----------------------------------------------------------------------

    @Override
    protected void updateDifferentialInputs(DifferentialMechanismInputsAutoLogged diffInputs) {
        // Recompute the combined axis positions from the simulated motor positions.
        // This mirrors how CTRE computes them internally for the real mechanism.
        Angle leaderPos = inputs.position;
        Angle followerPos = followerInputs.position;

        // Average axis = average of both motors
        diffInputs.averagePosition = leaderPos.plus(followerPos).times(0.5);
        diffInputs.averageVelocity =
                RotationsPerSecond.of(
                        (inputs.velocity.in(RotationsPerSecond)
                                        + followerInputs.velocity.in(RotationsPerSecond))
                                / 2.0);

        // Differential axis = half the difference of both motors
        diffInputs.differentialPosition = leaderPos.minus(followerPos).times(0.5);
        diffInputs.differentialVelocity =
                RotationsPerSecond.of(
                        (inputs.velocity.in(RotationsPerSecond)
                                        - followerInputs.velocity.in(RotationsPerSecond))
                                / 2.0);

        // Simulation never has hardware faults
        diffInputs.isDisabled = false;
        diffInputs.requiresUserAction = false;
        diffInputs.averageClosedLoopError = 0.0;
        diffInputs.differentialClosedLoopError = 0.0;
    }

    // -----------------------------------------------------------------------
    // Differential control methods (open-loop for simulation)
    // -----------------------------------------------------------------------

    /**
     * {@inheritDoc}
     *
     * <p>In simulation, sets the applied voltage on each motor independently. The average voltage
     * drives both motors in the same direction; the differential voltage opposes one side.
     *
     * <p>Specifically: leaderVoltage = avg + diff, followerVoltage = avg − diff.
     */
    @Override
    public void runVoltage(Voltage averageVoltage, Voltage differentialVoltage) {
        double avg = averageVoltage.in(Volts);
        double diff = differentialVoltage.in(Volts);
        // The leader receives (avg + diff) and the follower receives (avg - diff)
        io.runVoltage(Volts.of(avg + diff));
        followerIo.runVoltage(Volts.of(avg - diff));
    }

    /**
     * {@inheritDoc}
     *
     * <p>In simulation, delegates the average-axis position to the leader MotorIOSim and the
     * differential-axis position to the follower MotorIOSim using independent position controllers.
     */
    @Override
    public void runPosition(
            Angle averagePosition,
            Angle differentialPosition,
            PIDSlot averageSlot,
            PIDSlot differentialSlot) {
        // Convert back to per-motor targets: leaderTarget = avg + diff, followerTarget = avg - diff
        Angle leaderTarget = averagePosition.plus(differentialPosition);
        Angle followerTarget = averagePosition.minus(differentialPosition);
        io.runPosition(leaderTarget, averageSlot);
        followerIo.runPosition(followerTarget, differentialSlot);
    }

    /**
     * {@inheritDoc}
     *
     * <p>In simulation, runs the average-axis velocity on the leader and holds the differential
     * axis via position control on the follower.
     */
    @Override
    public void runVelocityAverage(
            AngularVelocity averageVelocity,
            Angle differentialPosition,
            PIDSlot averageSlot,
            PIDSlot differentialSlot) {
        io.runVelocity(averageVelocity, averageSlot);
        // Compute the follower target needed to hold the differential position
        Angle followerTarget = inputs.position.minus(differentialPosition.times(2.0));
        followerIo.runPosition(followerTarget, differentialSlot);
    }

    @Override
    public void runBrake() {
        io.runBrake();
        followerIo.runBrake();
    }

    @Override
    public void runCoast() {
        io.runCoast();
        followerIo.runCoast();
    }
}
