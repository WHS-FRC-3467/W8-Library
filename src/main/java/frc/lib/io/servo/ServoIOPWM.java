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

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;
import lombok.Getter;

/**
 * ServoIOPWM is an abstraction for WPILib's Servo class with Pulse Width Modulation signal control (PWM) and implements the ServoIO interface.
 * It provides methods to run the servo to a specified position.
 */
public class ServoIOPWM implements ServoIO {
    
    @Getter
    private final String name;
    private final Servo servo;

    public ServoIOPWM(String name, int channel) {
        this.name = name;
        servo = new Servo(channel);

        servo.getSpeed();
    }

    @Override
    public void updateInputs(ServoInputs inputs) {
        inputs.position = Degrees.of(servo.getAngle());
    }

    /** 
     * Set the servo position by specifying the angle, in degrees from 0 to 180. 
     * This method will work for servos with the same range as the Hitec HS-322HD servo . 
     * Any values passed to this method outside the specified range will be coerced to the boundary.
     */
    @Override
    public void runPosition(Angle position) {
        servo.setAngle(position.in(Degrees));
    }

    @Override
    public void runPosition(double value) {
        servo.set(MathUtil.clamp(value, 0.0, 1.0));
    }

    /**
     * Disables the PWM output until told to run to a position again.
     */
    public void stop() {
        servo.setDisabled();
    }
}
