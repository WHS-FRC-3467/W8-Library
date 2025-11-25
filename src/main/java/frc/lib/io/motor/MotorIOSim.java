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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Extension of MotorIO interface for simulated motors.
 * 
 * <p>
 * This interface adds methods specific to simulation, allowing WPILib physics simulations to feed
 * data back into the motor IO layer. This creates a realistic simulation where the simulated
 * physics affects the motor's sensor readings.
 */
public interface MotorIOSim extends MotorIO {
    /**
     * Sets the simulated mechanism position.
     * 
     * <p>
     * This is called by WPILib mechanism simulations (like SingleJointedArmSim) to update the
     * motor's position sensor based on the simulated physics. This creates a feedback loop where
     * motor output affects the simulation, and the simulation updates the sensors.
     * 
     * @param position The new mechanism position from the physics simulation
     */
    public default void setPosition(Angle position)
    {}

    /**
     * Sets the simulated motor rotor velocity.
     * 
     * <p>
     * The rotor is the spinning part inside the motor. This method updates the velocity sensor
     * reading based on what the physics simulation calculated. Rotor velocity is before any gear
     * reduction.
     * 
     * @param velocity The new motor rotor velocity from the physics simulation
     */
    public default void setRotorVelocity(AngularVelocity velocity)
    {}

    /**
     * Sets the simulated motor rotor acceleration.
     * 
     * <p>
     * This updates the acceleration value based on the physics simulation. Acceleration is useful
     * for advanced control algorithms and can help detect mechanism problems.
     * 
     * @param acceleration The new motor rotor acceleration from the physics simulation
     */
    public default void setRotorAcceleration(AngularAcceleration acceleration)
    {}

    /**
     * Gets the gear ratio from the motor rotor to the sensor.
     * 
     * <p>
     * This is the first stage of gearing, from the motor's internal rotor to wherever the encoder
     * is mounted. For example, if there's a 2:1 reduction between the motor and encoder, this
     * returns 2.0.
     * 
     * @return The rotor-to-sensor gear ratio
     */
    public default double getRotorToSensorRatio()
    {
        return 0.0;
    }

    /**
     * Gets the gear ratio from the sensor to the final mechanism.
     * 
     * <p>
     * This is the second stage of gearing, from the encoder to the actual mechanism you're
     * controlling. For example, if there's a 2:1 reduction between the encoder and your arm pivot,
     * this returns 2.0. The total reduction is rotor-to-sensor × sensor-to-mechanism.
     * 
     * @return The sensor-to-mechanism gear ratio
     */
    public default double getSensorToMechanismRatio()
    {
        return 0.0;
    }

    /**
     * Closes and cleans up simulation resources.
     * 
     * <p>
     * Called when the simulation is shutting down to properly release any resources.
     */
    @Override
    public default void close()
    {}
}
