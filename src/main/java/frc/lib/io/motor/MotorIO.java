/*
 * Copyright (C) 2025 Windham Windup
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

package frc.lib.io.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import lombok.AllArgsConstructor;
import lombok.Getter;

/**
 * Standardized interface for motor controllers used in FRC. Supports multiple control modes and
 * telemetry reporting.
 */
public interface MotorIO {

    @Getter
    @AllArgsConstructor
    public enum PIDSlot {
        SLOT_0(0),
        SLOT_1(1),
        SLOT_2(2);

        public final int num;
    }

    public enum ControlType {
        COAST,
        BRAKE,
        VOLTAGE,
        CURRENT,
        DUTYCYCLE,
        POSITION,
        VELOCITY
    }

    @AutoLog
    abstract class MotorInputs {
        /** Whether the motor is connected. */
        public boolean connected = false;
        /** Motor position. */
        public Angle position = Radians.of(0.0);
        /** Motor velocity. */
        public AngularVelocity velocity = RadiansPerSecond.of(0.0);
        /** Voltage applied to the motor. */
        public Voltage appliedVoltage = Volts.of(0.0);
        /** Total supply current to the motor. */
        public Current supplyCurrent = Amps.of(0.0);
        /** Torque-producing current. */
        public Current torqueCurrent = Amps.of(0.0);
        /** Motor temperature in degrees. */
        public Temperature temperature = Celsius.of(0.0);
        /** Error in position */
        public Angle positionError = null;
        /** Error in velocity */
        public AngularVelocity velocityError = null;
        /** Active trajectory position in rotations */
        public Angle activeTrajectoryPosition = null;
        /** Active trajectory velocity in rotations per second. */
        public AngularVelocity activeTrajectoryVelocity = null;
        /** Current control type */
        public ControlType controlType = null;
    }

    /**
     * Getter for the name of the motor
     * 
     * @return The name of the motor
     */
    public default String getName()
    {
        return "";
    }

    /**
     * Updates the provided {@link MotorInputs} instance with the latest sensor readings. If the
     * sensor is not connected, it populates the fields with default values.
     *
     * @param inputs The structure to populate with updated sensor values.
     */
    public default void updateInputs(MotorInputs inputs)
    {}

    /**
     * Sets the motor to coast mode.
     */
    public default void runCoast()
    {}

    /**
     * Sets the motor to brake mode.
     */
    public default void runBrake()
    {}

    /**
     * Runs the motor using direct voltage control.
     *
     * @param voltage Desired voltage output.
     */
    public default void runVoltage(Voltage voltage)
    {}

    /**
     * Runs the motor with a specified current output.
     *
     * @param current Desired torque-producing current.
     */
    public default void runCurrent(Current current)
    {}

    /**
     * Runs the motor with a specified current output and duty cycle.
     *
     * @param current Desired torque-producing current.
     * @param dutyCycle Desired dutycycle of current output, limiting top speed
     */
    public default void runCurrent(Current current, double dutyCycle)
    {}

    /**
     * Runs the motor using duty cycle (percentage of available voltage).
     *
     * @param dutyCycle Fractional output between 0 and 1.
     */
    public default void runDutyCycle(double dutyCycle)
    {}

    /**
     * Runs the motor to a specific position.
     *
     * @param position Target position.
     * @param cruiseVelocity Cruise velocity.
     * @param acceleration Max acceleration.
     * @param maxJerk Max jerk (rate of acceleration).
     * @param slot PID slot index.
     */
    public default void runPosition(Angle position, AngularVelocity cruiseVelocity,
        AngularAcceleration acceleration,
        Velocity<AngularAccelerationUnit> maxJerk, PIDSlot slot)
    {}

    /**
     * Runs the motor at a target velocity.
     *
     * @param velocity Desired velocity.
     * @param acceleration Max acceleration.
     * @param slot PID slot index.
     */
    public default void runVelocity(AngularVelocity velocity, AngularAcceleration acceleration,
        PIDSlot slot)
    {}

    /**
     * Sets the position of the motor's internal encoder
     * 
     * @param position Desired position to set encoder to
     */
    public default void setEncoderPosition(Angle position)
    {}
}
