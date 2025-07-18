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

package frc.robot.subsystems.servo1;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.servo.ServoIO;
import frc.lib.subsystems.Servo;
import org.littletonrobotics.junction.Logger;

public class Servo1 extends SubsystemBase {
    
    public enum State {
        IDLE,
        RETRACTED,
        EXTENDED
    }

    State state = State.IDLE;

    private final Servo servo;

    public Servo1(ServoIO io) {
        servo = new Servo(io);
    }

    @Override
    public void periodic() {
        Logger.recordOutput(Servo1Constants.NAME + "/state", state.toString());	
    }

    public void setState(State state) {
        switch (state) {
            case IDLE:
                servo.stop();
                break;
            case RETRACTED:
                servo.setAngle(Servo1Constants.RETRACTED_ANGLE);
                break;
            case EXTENDED:
                servo.setAngle(Servo1Constants.EXTENDED_ANGLE);
                break;
        }
        this.state = state;
    }
}
