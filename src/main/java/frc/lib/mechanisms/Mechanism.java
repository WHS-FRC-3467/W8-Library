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

package frc.lib.mechanisms;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.io.motor.MotorIO.PIDSlot;

public interface Mechanism {

    /** Call this method periodically */
    public default void periodic()
    {}

    /**
     * Sets the mechanism to coast mode.
     */
    public default void runCoast()
    {}

    /**
     * Sets the mechanism to brake mode.
     */
    public default void runBrake()
    {}

    /**
     * Runs the mechanism using direct voltage control.
     *
     * @param voltage Desired voltage output.
     */
    public default void runVoltage(Voltage voltage)
    {}

    /**
     * Runs the mechanism with a specified current output.
     *
     * @param current Desired torque-producing current.
     */
    public default void runCurrent(Current current)
    {}

    /**
     * Runs the mechanism using duty cycle (percentage of available voltage).
     *
     * @param dutyCycle Fractional output between 0 and 1.
     */
    public default void runDutyCycle(double dutyCycle)
    {}

    /**
     * Runs the mechanism to a specific position.
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
     * Runs the mechanism at a target velocity.
     *
     * @param velocity Desired velocity.
     * @param acceleration Max acceleration.
     * @param slot PID slot index.
     */
    public default void runVelocity(AngularVelocity velocity, AngularAcceleration acceleration,
        PIDSlot slot)
    {}
}
