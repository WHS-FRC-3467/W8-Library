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

package frc.robot.subsystems.intake;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.Radians;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import frc.lib.actuators.linear.LinearActuator;
import frc.lib.actuators.linear.LinearActuator.LinearMechCharacteristics;
import frc.lib.actuators.linear.LinearActuatorReal;
import frc.lib.actuators.linear.LinearActuatorSim;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.io.motor.MotorIOTalonFX;
import frc.lib.io.motor.MotorIOTalonFXSim;
import frc.lib.util.PID;
import frc.robot.Constants;
import frc.robot.Ports;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;

import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearAcceleration;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Mass;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class IntakeLinearConstants {

    public static final String NAME = "Intake Linear";

    public static final double GEARING = (42.0 / 12.0);

    public static final Distance MIN_DISTANCE = Inches.of(0.0);
    public static final Distance MAX_DISTANCE = Inches.of(11.1);
    public static final Distance STARTING_DISTANCE = Inches.of(0.1);

    public static final LinearVelocity CRUISE_VELOCITY = MetersPerSecond.of(1.0);

    public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(999.0);

    public static final Distance DRUM_RADIUS = Inches.of(0.511);
    public static final Mass CARRIAGE_MASS = Kilograms.of(.1);

    public static final Distance TOLERANCE = Inches.of(2);

    public static final DCMotor DCMOTOR = DCMotor.getKrakenX44Foc(1);

    // Orientation for the linear mechanism.
    // Uses WPILib's counter-clockwise positive convention around Y-axis:
    // A pitch of -90 degrees represents a vertical mechanism extending upward (like an elevator).
    // Pitch of 0 degrees would be horizontal extending forward.
    // Roll and yaw can be used for mechanisms that extend in other directions.
    public static final Rotation3d ORIENTATION =
            new Rotation3d(0.0, Degrees.of(0.0).in(Radians), 0.0);

    public static final LinearMechCharacteristics CHARACTERISTICS =
            new LinearMechCharacteristics(
                    MIN_DISTANCE, MAX_DISTANCE, STARTING_DISTANCE, DRUM_RADIUS, ORIENTATION);

    public static final PID SLOT0_PID =
            RobotBase.isSimulation()
                    ? new PID(10.0, 0.0, 0.0)
                    : new PID(30.0, 0.0, 0.0).withS(14.0);

    /**
     * Creates and configures a TalonFX motor controller configuration for the intake linear
     * mechanism.
     *
     * @return The configured TalonFX configuration
     */
    public static TalonFXConfiguration getFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = false;

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        config.TorqueCurrent.PeakForwardTorqueCurrent = 50.0;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -50.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted =
                RobotBase.isSimulation()
                        ? InvertedValue.CounterClockwise_Positive
                        : InvertedValue.Clockwise_Positive;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                Units.radiansToRotations(MAX_DISTANCE.in(Meters) / DRUM_RADIUS.in(Meters));

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                Units.radiansToRotations(MIN_DISTANCE.in(Meters) / DRUM_RADIUS.in(Meters));

        config.MotionMagic.MotionMagicCruiseVelocity =
                Units.radiansToRotations(
                        CRUISE_VELOCITY.in(MetersPerSecond) / DRUM_RADIUS.in(Meters));
        config.MotionMagic.MotionMagicAcceleration =
                Units.radiansToRotations(
                        MAX_ACCELERATION.in(MetersPerSecondPerSecond) / DRUM_RADIUS.in(Meters));

        config.Feedback.RotorToSensorRatio = 1.0;

        config.Feedback.SensorToMechanismRatio = GEARING;

        config.Slot0 = Slot0Configs.from(SLOT0_PID.toSlotConfigs());

        return config;
    }

    /**
     * Factory method to create a LinearMechanism instance for the intake linear subsystem. Creates
     * the appropriate mechanism based on the current robot mode (REAL, SIM, or REPLAY).
     *
     * @return A configured LinearMechanism instance
     */
    public static LinearActuator<?> getMechanism() {
        LinearActuator<?> mechanism;
        switch (Constants.CURRENT_MODE) {
            case REAL:
                mechanism =
                        new LinearActuatorReal(
                                NAME,
                                new MotorIOTalonFX(NAME, getFXConfig(), Ports.intakeLinear),
                                CHARACTERISTICS);
                break;
            case SIM:
                mechanism =
                        new LinearActuatorSim(
                                NAME,
                                new MotorIOTalonFXSim(
                                        NAME,
                                        MotorType.KrakenX44,
                                        DCMOTOR,
                                        getFXConfig(),
                                        Ports.intakeLinear),
                                DCMOTOR,
                                GEARING,
                                1,
                                CARRIAGE_MASS,
                                false,
                                CHARACTERISTICS);
                break;
            case REPLAY:
                mechanism = new LinearActuator<>(NAME, CHARACTERISTICS, new MotorIO() {}) {};
                break;
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }
        mechanism.enableTunablePID(PIDSlot.SLOT_0, SLOT0_PID);
        mechanism.withRadius(DRUM_RADIUS);
        return mechanism;
    }
}
