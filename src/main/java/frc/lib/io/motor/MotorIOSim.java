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

package frc.lib.io.motor;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public interface MotorIOSim extends MotorIO {
    /**
     * Setter for the position of the mechanism associated with this motor group, typically taken from a WPILib mechanism
     * simulation
     *
     * @param position The new mechanism position (in mechanism-space)
     */
    public default void setMechanismPosition(Angle position) {}

    /**
     * Setter for the velocity of the mechanism associated with this motor group, typically taken from a WPILib mechanism
     * simulation
     *
     * @param velocity The new mechanism velocity (in mechanism-space)
     */
    public default void setMechanismVelocity(AngularVelocity velocity) {}

    /**
     * Setter for the acceleration of the mechanism associated with this motor group, typically taken from a WPILib mechanism
     * simulation
     *
     * @param acceleration The new mechanism acceleration (in mechanism-space)
     */
    public default void setMechanismAcceleration(AngularAcceleration acceleration) {}
}
