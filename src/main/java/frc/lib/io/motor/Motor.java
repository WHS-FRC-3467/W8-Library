// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.lib.util.CANDevice;

/**
 * Standardized interface for motor controllers used in FRC. Supports multiple control modes,
 * telemetry reporting, and follower configuration.
 */
public interface Motor {

    public enum PIDSlot {
        SLOT_1,
        SLOT_2,
        SLOT_3;
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
    }

    /**
     * Returns the motor's CAN device ID.
     *
     * @return The CANDevice representing this motor's ID and bus name.
     */
    public CANDevice getID();

    /**
     * Updates the provided {@link MotorInputs} instance with the latest sensor readings. If the
     * sensor is not connected, it populates the fields with default values.
     *
     * @param inputs The structure to populate with updated sensor values.
     */
    public void updateInputs(MotorInputs inputs);

    /**
     * Sets the motor to coast mode.
     */
    public void runCoast();

    /**
     * Sets the motor to brake mode.
     */
    public void runBrake();

    /**
     * Follows the specified motor using CAN follower mode.
     *
     * @param followMotor The motor to follow.
     * @param oppose Whether or not to oppose the main motor.
     */
    public void follow(Motor motor, boolean oppose);

    /**
     * Runs the motor using direct voltage control.
     *
     * @param voltage Desired voltage output.
     */
    public void runVoltage(Voltage voltage);

    /**
     * Runs the motor with a specified current output.
     *
     * @param current Desired torque-producing current.
     */
    public void runCurrent(Current current);

    /**
     * Runs the motor using duty cycle (percentage of available voltage).
     *
     * @param dutyCycle Fractional output between 0 and 1.
     */
    public void runDutyCycle(double dutyCycle);

    /**
     * Runs the motor to a specific position.
     *
     * @param position Target position.
     * @param cruiseVelocity Cruise velocity.
     * @param acceleration Max acceleration.
     * @param maxJerk Max jerk (rate of acceleration).
     * @param slot PID slot index.
     */
    public void runPosition(Angle position, AngularVelocity cruiseVelocity,
        AngularAcceleration acceleration,
        Velocity<AngularAccelerationUnit> maxJerk, PIDSlot slot);

    /**
     * Runs the motor at a target velocity.
     *
     * @param velocity Desired velocity.
     * @param acceleration Max acceleration.
     * @param slot PID slot index.
     */
    public void runVelocity(AngularVelocity velocity, AngularAcceleration acceleration,
        PIDSlot slot);
}
