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

import java.util.function.Supplier;
import org.apache.commons.lang3.NotImplementedException;
import edu.wpi.first.math.geometry.Pose3d;
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
    {
        throw new NotImplementedException("This method has not been implemented");
    }

    /**
     * Sets the mechanism to coast mode.
     */
    public default void runCoast()
    {
        throw new NotImplementedException("This method has not been implemented");
    }

    /**
     * Sets the mechanism to brake mode.
     */
    public default void runBrake()
    {
        throw new NotImplementedException("This method has not been implemented");
    }

    /**
     * Runs the mechanism using direct voltage control.
     *
     * @param voltage Desired voltage output.
     */
    public default void runVoltage(Voltage voltage)
    {
        throw new NotImplementedException("This method has not been implemented");
    }

    /**
     * Runs the mechanism with a specified current output.
     *
     * @param current Desired torque-producing current.
     */
    public default void runCurrent(Current current)
    {
        throw new NotImplementedException("This method has not been implemented");
    }

    /**
     * Runs the mechanism using duty cycle (percentage of available voltage).
     *
     * @param dutyCycle Fractional output between 0 and 1.
     */
    public default void runDutyCycle(double dutyCycle)
    {
        throw new NotImplementedException("This method has not been implemented");
    }

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
    {
        throw new NotImplementedException("This method has not been implemented");
    }

    /**
     * Runs the mechanism to a specific position.
     *
     * @param position Target position.
     * @param slot PID slot index.
     */
    public default void runPosition(Angle position, PIDSlot slot)
    {
        throw new NotImplementedException("This method has not been implemented");
    }

    /**
     * Runs the mechanism at a target velocity.
     *
     * @param velocity Desired velocity.
     * @param acceleration Max acceleration.
     * @param slot PID slot index.
     */
    public default void runVelocity(AngularVelocity velocity, AngularAcceleration acceleration,
        PIDSlot slot)
    {
        throw new NotImplementedException("This method has not been implemented");
    }

    /**
     * Runs the mechanism at a target velocity.
     *
     * @param velocity Desired velocity.
     * @param slot PID slot index.
     */
    public default void runVelocity(AngularVelocity velocity, PIDSlot slot)
    {
        throw new NotImplementedException("This method has not been implemented");
    }

    /**
     * Sets the position of the motor's internal encoder
     * 
     * @param position Desired position to set encoder to
     */
    public default void setEncoderPosition(Angle position)
    {
        throw new NotImplementedException("This method has not been implemented");
    }

    public default Current getSupplyCurrent()
    {
        throw new NotImplementedException("This method has not been implemented");
    }

    /**
     * Getter for angle of the motor
     * 
     * @return Angle of the motor or fused encoder
     */
    public default Angle getPosition()
    {
        throw new NotImplementedException("This method has not been implemented");
    }

    public default Current getTorqueCurrent()
    {
        throw new NotImplementedException("This method has not been implemented");
    }

    public default AngularVelocity getVelocity()
    {
        throw new NotImplementedException("This method has not been implemented");
    }

    public default void close()
    {
        throw new NotImplementedException("This method has not been implemented");
    }

    /**
     * Supplier for the Pose3d of the mechanism
     * 
     * @return Supplier for the Pose3d
     */
    public default Supplier<Pose3d> getPoseSupplier()
    {
        throw new NotImplementedException("This method has not been implemented");
    }


}
