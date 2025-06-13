// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.io.motor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** Add your docs here. */
public abstract class MotorIO {

    public class Inputs {
    }

    public sealed interface Setpoint {
        public record Idle() implements Setpoint {
        }

        public record Voltage(edu.wpi.first.units.measure.Voltage volts) implements Setpoint {
        }

        public record DutyCycle(double dutyCycle) implements Setpoint {
            public DutyCycle {
                dutyCycle = MathUtil.clamp(dutyCycle, 0, 1);
            }
        }

        public record Position(Angle position) implements Setpoint {
        }

        public record Velocity(AngularVelocity velocity) implements Setpoint {
        }

        public record MotionProfiledPosition(Angle position) implements Setpoint {
        }

        public record MotionProfiledVelocity(AngularVelocity velocity) implements Setpoint {
        }
    }

}
