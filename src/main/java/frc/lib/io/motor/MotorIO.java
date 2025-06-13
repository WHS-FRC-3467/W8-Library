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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public abstract class MotorIO {

    @AutoLog
    abstract class MotorIOInputs {
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
        /** Error in position (e.g., setpoint - actual). */
        public Angle positionError = null;
        /** Error in velocity (e.g., setpoint - actual). */
        public AngularVelocity velocityError = null;
        /**
         * Active trajectory position in rotations (from a motion profile or
         * trajectory).
         */
        public Angle activeTrajectoryPosition = null;
        /** Active trajectory velocity in rotations per second. */
        public AngularVelocity activeTrajectoryVelocity = null;
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
