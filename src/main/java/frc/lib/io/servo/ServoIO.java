/* Copyright (C) 2025 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

package frc.lib.io.servo;

import static edu.wpi.first.units.Units.Radians;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Angle;

public interface ServoIO {
    
    @AutoLog
    abstract class ServoInputs 
    {
        /** Servo position. */
        public Angle position = Radians.of(0.0);
    }

    /**
     * Getter for the name of the servo
     * 
     * @return The name of the servo
     */
    public default String getName()
    {
        return "";
    }

    /**
     * Updates the provided {@link ServoInputs} instance with the latest sensor readings.
     *
     * @param inputs The structure to populate with updated sensor values.
     */
    public default void updateInputs(ServoInputs inputs)
    {}

    /**
     * Runs the servo to position using a scaled 0 to 1.0 value. 
     * 0.0 corresponds to one extreme of the servo and 1.0 corresponds to the other.
     * @param value
     */
    public default void runPosition(double value) {}

    /**
     * Runs the servo to position using an {@link Angle} value.
     * The value should not exceed the lower and upper limits of the servo.
     * @param position
     */
    public default void runPosition(Angle position) {}

    /** Runs the servo at neutral mode. */
    public default void stop() {}
}
