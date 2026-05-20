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

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import frc.lib.io.absoluteencoder.AbsoluteEncoderIO;
import frc.lib.io.absoluteencoder.AbsoluteEncoderInputsAutoLogged;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.io.motor.MotorInputsAutoLogged;
import frc.lib.mechanisms.Mechanism;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

/**
 * Abstract base class for two-motor differential mechanisms. A differential mechanism has two axes:
 * an <b>average axis</b> (driven by both motors together) and a <b>differential axis</b> (driven by
 * the difference between the two motors). Common examples include differential wrists and
 * differential shooters.
 *
 * <p>This class wraps two {@link MotorIO} instances (leader and follower) and logs both
 * individually, as well as the combined average/differential signals.
 *
 * @param <T> the MotorIO type for the leader motor
 * @param <S> the MotorIO type for the follower motor
 * @param <E> the AbsoluteEncoderIO type (may be unused / Optional.empty())
 */
public abstract class DifferentialMechanism<
                T extends MotorIO, S extends MotorIO, E extends AbsoluteEncoderIO>
        extends Mechanism<T> {

    // -----------------------------------------------------------------------
    // Enums describing which physical axis each motor combination controls
    // -----------------------------------------------------------------------

    /** The physical axis controlled by the <em>difference</em> between the two motors. */
    public enum DifferenceAxis {
        PITCH,
        YAW,
        ROLL,
        X,
        Y,
        Z
    }

    /** The physical axis controlled by the <em>average</em> of the two motors. */
    public enum AverageAxis {
        PITCH,
        YAW,
        ROLL,
        X,
        Y,
        Z
    }

    // -----------------------------------------------------------------------
    // Mechanism characteristics record
    // -----------------------------------------------------------------------

    /**
     * Immutable description of this differential mechanism's geometry and starting state.
     *
     * @param differenceAxis the axis driven by the difference of the two motors
     * @param averageAxis the axis driven by the average of the two motors
     * @param gearing gear ratio from motor to mechanism output
     * @param startingDifference initial differential axis position
     * @param startingAverage initial average axis position
     */
    public static record DiffMechCharacteristics(
            DifferenceAxis differenceAxis,
            AverageAxis averageAxis,
            double gearing,
            Angle startingDifference,
            Angle startingAverage) {}

    // -----------------------------------------------------------------------
    // Differential-specific logged inputs (average & differential axes)
    // -----------------------------------------------------------------------

    /**
     * AdvantageKit-logged inputs for the combined average/differential axis signals. These are
     * separate from each motor's individual inputs and represent the mechanism as a whole.
     */
    @AutoLog
    public abstract static class DifferentialMechanismInputs {
        /** Average (sum) axis position — controlled by driving both motors together. */
        public Angle averagePosition = Rotations.zero();

        /** Average axis velocity. */
        public AngularVelocity averageVelocity = RotationsPerSecond.zero();

        /** Differential axis position — controlled by the difference between the two motors. */
        public Angle differentialPosition = Rotations.zero();

        /** Differential axis velocity. */
        public AngularVelocity differentialVelocity = RotationsPerSecond.zero();

        /** Closed-loop error on the average axis (in rotations or native units). */
        public double averageClosedLoopError = 0.0;

        /** Closed-loop error on the differential axis. */
        public double differentialClosedLoopError = 0.0;

        /** Closed-loop reference (target) on the average axis. */
        public double averageClosedLoopReference = 0.0;

        /** Closed-loop reference (target) on the differential axis. */
        public double differentialClosedLoopReference = 0.0;

        /** True when the mechanism has been automatically disabled due to a fault. */
        public boolean isDisabled = false;

        /** True when the mechanism requires explicit user action before it can resume. */
        public boolean requiresUserAction = false;
    }

    // -----------------------------------------------------------------------
    // Fields
    // -----------------------------------------------------------------------

    /** Follower motor IO (the second gearbox). */
    protected final S followerIo;

    /** Logged inputs for the follower motor. */
    protected final MotorInputsAutoLogged followerInputs = new MotorInputsAutoLogged();

    /** Logged inputs for the combined differential signals. */
    protected final DifferentialMechanismInputsAutoLogged diffMechInputs =
            new DifferentialMechanismInputsAutoLogged();

    /** Optional absolute encoder (e.g., CANcoder on the differential axis). */
    protected final Optional<E> absoluteEncoder;

    /** Visualizer for the average and differential axes. */
    protected final DifferentialVisualizer visualizer;

    private final String encoderName;
    protected final AbsoluteEncoderInputsAutoLogged absoluteEncoderInputs =
            new AbsoluteEncoderInputsAutoLogged();

    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    protected DifferentialMechanism(
            String name,
            DiffMechCharacteristics characteristics,
            T leaderIo,
            S followerIo,
            Optional<E> absoluteEncoder,
            String encoderName) {
        super(name, leaderIo);
        this.followerIo = followerIo;
        this.absoluteEncoder = absoluteEncoder;
        this.encoderName = encoderName;
        this.visualizer = new DifferentialVisualizer(name, characteristics);
    }

    // -----------------------------------------------------------------------
    // Abstract method — subclasses fill in combined axis telemetry
    // -----------------------------------------------------------------------

    /**
     * Subclasses must implement this to populate the {@link DifferentialMechanismInputsAutoLogged}
     * with the latest average/differential position, velocity, and closed-loop signals.
     *
     * @param inputs the inputs object to fill in
     */
    protected abstract void updateDifferentialInputs(DifferentialMechanismInputsAutoLogged inputs);

    // -----------------------------------------------------------------------
    // Differential control methods — override in subclasses for real hardware
    // -----------------------------------------------------------------------

    /**
     * Runs both axes using open-loop voltage control. The average voltage drives both motors in the
     * same direction; the differential voltage drives them in opposite directions.
     *
     * @param averageVoltage voltage applied to the average (common) axis
     * @param differentialVoltage voltage applied to the differential axis
     */
    public void runVoltage(Voltage averageVoltage, Voltage differentialVoltage) {}

    /**
     * Runs the average axis to a position setpoint and holds the differential axis at a fixed
     * position using closed-loop control. Motion Magic® is applied on the average axis.
     *
     * @param averagePosition target position for the average axis
     * @param differentialPosition target position for the differential axis
     * @param averageSlot PID slot to use on the average axis (typically Slot 0)
     * @param differentialSlot PID slot to use on the differential axis (typically Slot 1)
     */
    public void runPosition(
            Angle averagePosition,
            Angle differentialPosition,
            PIDSlot averageSlot,
            PIDSlot differentialSlot) {}

    /**
     * Runs the average axis at a target velocity while holding the differential axis at a fixed
     * position.
     *
     * @param averageVelocity target velocity for the average axis
     * @param differentialPosition target position for the differential axis
     * @param averageSlot PID slot for velocity control on the average axis
     * @param differentialSlot PID slot for position control on the differential axis
     */
    public void runVelocityAverage(
            AngularVelocity averageVelocity,
            Angle differentialPosition,
            PIDSlot averageSlot,
            PIDSlot differentialSlot) {}

    /**
     * Requests neutral (brake or coast, depending on configuration) output from both motors.
     * Overrides the base class single-motor stop so both sides stop together.
     */
    @Override
    public void runBrake() {
        // Default: delegate to the single-motor path for the leader.
        // DifferentialMechanismReal overrides this to stop both via CTRE.
        super.runBrake();
    }

    /**
     * Requests coast output from both motors. Overrides the base class single-motor coast so both
     * sides coast together.
     */
    @Override
    public void runCoast() {
        super.runCoast();
    }

    // -----------------------------------------------------------------------
    // Getters for current state
    // -----------------------------------------------------------------------

    /** Returns the most recently logged average-axis position. */
    public Angle getAveragePosition() {
        return diffMechInputs.averagePosition;
    }

    /** Returns the most recently logged differential-axis position. */
    public Angle getDifferentialPosition() {
        return diffMechInputs.differentialPosition;
    }

    /** Returns the most recently logged average-axis velocity. */
    public AngularVelocity getAverageVelocity() {
        return diffMechInputs.averageVelocity;
    }

    /** Returns the most recently logged differential-axis velocity. */
    public AngularVelocity getDifferentialVelocity() {
        return diffMechInputs.differentialVelocity;
    }

    /** Returns {@code true} if the mechanism is currently disabled due to a fault. */
    public boolean isDisabled() {
        return diffMechInputs.isDisabled;
    }

    /** Returns {@code true} if the mechanism requires user action to re-enable. */
    public boolean requiresUserAction() {
        return diffMechInputs.requiresUserAction;
    }

    // -----------------------------------------------------------------------
    // Periodic
    // -----------------------------------------------------------------------

    @Override
    public void periodic() {
        // 1. Update and log the leader motor inputs (handled by Mechanism<T>).
        super.periodic();

        // 2. Update and log the follower motor inputs.
        followerIo.updateInputs(followerInputs);
        Logger.processInputs(name + "/Follower", followerInputs);

        // 3. Update and log the combined differential axis inputs from this mechanism.
        updateDifferentialInputs(diffMechInputs);
        Logger.processInputs(name + "/DifferentialAxes", diffMechInputs);

        // 4. Update and log the optional absolute encoder.
        absoluteEncoder.ifPresent(
                encoder -> {
                    encoder.updateInputs(absoluteEncoderInputs);
                    Logger.processInputs(encoderName, absoluteEncoderInputs);
                });

        // 5. Update the 2D visualizer with the latest axis positions.
        visualizer.setAverageAngle(diffMechInputs.averagePosition);
        visualizer.setDifferentialAngle(diffMechInputs.differentialPosition);
    }

    // -----------------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------------

    @Override
    public void close() {
        super.close();
        followerIo.close();
        absoluteEncoder.ifPresent(AbsoluteEncoderIO::close);
    }
}
