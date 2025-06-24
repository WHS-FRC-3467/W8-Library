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

package frc.lib.subsystems;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.io.motor.MotorInputsAutoLogged;

/**
 * Class for simplified MotorIO implementation
 */
public class Motor {
    private final MotorIO io;
    private final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();

    /**
     * Constructs a Motor.
     *
     * @param io the IO to interact with.
     */
    public Motor(MotorIO io)
    {
        this.io = io;
    }

    /** Call this method periodically */
    public void periodic()
    {
        io.updateInputs(inputs);
        Logger.processInputs(io.getName(), inputs);
    }

    /**
     * Sets the motor to coast mode.
     */
    public void runCoast()
    {
        io.runCoast();
    }

    /**
     * Sets the motor to brake mode.
     */
    public void runBrake()
    {
        io.runBrake();
    }

    /**
     * Follows the specified motor using CAN follower mode.
     *
     * @param followMotor The motor to follow.
     * @param oppose Whether or not to oppose the main motor.
     */
    public void follow(MotorIO motor, boolean oppose)
    {
        io.follow(motor, oppose);
    }

    /**
     * Runs the motor using direct voltage control.
     *
     * @param voltage Desired voltage output.
     */
    public void runVoltage(Voltage voltage)
    {
        io.runVoltage(voltage);
    }

    /**
     * Runs the motor with a specified current output.
     *
     * @param current Desired torque-producing current.
     */
    public void runCurrent(Current current)
    {
        io.runCurrent(current);
    }

    /**
     * Runs the motor using duty cycle (percentage of available voltage).
     *
     * @param dutyCycle Fractional output between 0 and 1.
     */
    public void runDutyCycle(double dutyCycle)
    {
        io.runDutyCycle(dutyCycle);
    }

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
        Velocity<AngularAccelerationUnit> maxJerk, PIDSlot slot)
    {
        io.runPosition(position, cruiseVelocity, acceleration, maxJerk, slot);
    }

    /**
     * Runs the motor at a target velocity.
     *
     * @param velocity Desired velocity.
     * @param acceleration Max acceleration.
     * @param slot PID slot index.
     */
    public void runVelocity(AngularVelocity velocity, AngularAcceleration acceleration,
        PIDSlot slot)
    {
        io.runVelocity(velocity, acceleration, slot);
    }
}
