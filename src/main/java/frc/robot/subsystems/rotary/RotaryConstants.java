// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rotary;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import java.util.Optional;
import static edu.wpi.first.units.Units.Meters;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.io.absoluteencoder.AbsoluteEncoderIOCANCoderSim;
import frc.lib.io.motor.MotorIOTalonFX;
import frc.lib.io.motor.MotorIOTalonFX.TalonFXFollower;
import frc.lib.io.motor.MotorIOTalonFXSim;
import frc.lib.mechanisms.rotary.*;
import frc.lib.mechanisms.rotary.RotaryMechanism.RotaryAxis;
import frc.lib.mechanisms.rotary.RotaryMechanism.RotaryMechCharacteristics;
import frc.robot.Constants;
import frc.robot.Ports;


public class RotaryConstants {
    public static String NAME = "Rotary";

    public static final Angle TOLERANCE = Degrees.of(2.0);

    public static final AngularVelocity CRUISE_VELOCITY =
        Units.RadiansPerSecond.of(1);
    public static final AngularAcceleration ACCELERATION =
        CRUISE_VELOCITY.div(0.1).per(Units.Second);
    public static final Velocity<AngularAccelerationUnit> JERK = ACCELERATION.per(Second);

    private static final double ROTOR_TO_SENSOR = (2.0 / 1.0);
    private static final double SENSOR_TO_MECHANISM = (2.0 / 1.0);

    public static final Translation3d OFFSET = Translation3d.kZero;

    public static final Angle MIN_ANGLE = Degrees.of(-130.0);
    public static final Angle MAX_ANGLE = Rotations.of(.5);
    public static final Angle STARTING_ANGLE = Rotations.of(0.0);
    public static final Distance ARM_LENGTH = Meters.of(1.0);

    public static final RotaryMechCharacteristics CONSTANTS =
        new RotaryMechCharacteristics(
            OFFSET,
            ARM_LENGTH,
            MIN_ANGLE,
            MAX_ANGLE,
            STARTING_ANGLE,
            RotaryAxis.PITCH);

    public static final Mass ARM_MASS = Kilograms.of(.01);
    public static final DCMotor DCMOTOR = DCMotor.getKrakenX60(1);
    public static final MomentOfInertia MOI = KilogramSquareMeters
        .of(SingleJointedArmSim.estimateMOI(ARM_LENGTH.in(Meters), ARM_MASS.in(Kilograms)));

    private static final Angle ENCODER_OFFSET = Rotations.of(0.0);

    public static final Rotary.Setpoint DEFAULT_SETPOINT = Rotary.Setpoint.STOW;

    // Positional PID
    private static Slot0Configs SLOT0CONFIG = new Slot0Configs()
        .withKP(30.0)
        .withKI(0.0)
        .withKD(5.0);

    /**
     * Creates and returns the TalonFX motor controller configuration for the rotary mechanism.
     * 
     * <p>
     * This configuration includes:
     * <ul>
     * <li>Current limits to prevent motor damage and brownouts</li>
     * <li>Voltage limits for power output</li>
     * <li>Brake mode to hold position when not moving</li>
     * <li>Software limit switches to prevent mechanism damage</li>
     * <li>Gear ratios for proper position/velocity feedback</li>
     * <li>Remote CANcoder feedback for absolute positioning</li>
     * <li>PID gains for control</li>
     * </ul>
     * 
     * @return A configured TalonFXConfiguration object ready to apply to a motor controller
     */
    public static TalonFXConfiguration getFXConfig()
    {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        config.CurrentLimits.StatorCurrentLimitEnable = false;
        config.CurrentLimits.StatorCurrentLimit = 80.0;

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE.in(Units.Rotations);

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE.in(Units.Rotations);

        config.Feedback.RotorToSensorRatio = ROTOR_TO_SENSOR;
        config.Feedback.SensorToMechanismRatio = SENSOR_TO_MECHANISM;

        config.Feedback.FeedbackRemoteSensorID = Ports.RotarySubsystemEncoder.id();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        config.Slot0 = SLOT0CONFIG;

        return config;
    }

    /**
     * Creates and returns the CANcoder absolute encoder configuration.
     * 
     * <p>
     * The CANcoder provides absolute position feedback, meaning it knows the true position of the
     * mechanism even after power cycling. The magnet offset calibrates the encoder's zero position.
     * 
     * @param sim Whether this configuration is for simulation (true) or real robot (false). In
     *        simulation, the offset is set to 0.0 since it's not needed.
     * @return A configured CANcoderConfiguration object
     */
    public static CANcoderConfiguration getCANcoderConfig(boolean sim)
    {
        CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.MagnetOffset = sim ? 0.0 : ENCODER_OFFSET.in(Rotations);

        return config;
    }

    public static Rotary get()
    {
        switch (Constants.currentMode) {
            case REAL:
                return new Rotary(new RotaryMechanismReal(
                    new MotorIOTalonFX(NAME, getFXConfig(), Ports.RotarySubsystemMotorMain,
                        new TalonFXFollower(Ports.RotarySubsystemMotorFollower, false)),
                    CONSTANTS,
                    Optional.of(new AbsoluteEncoderIOCANCoderSim(Ports.RotarySubsystemEncoder,
                        NAME + "Encoder", getCANcoderConfig(false)))));
            case SIM:
                return new Rotary(new RotaryMechanismSim(
                    new MotorIOTalonFXSim(NAME, getFXConfig(), Ports.RotarySubsystemMotorMain,
                        new TalonFXFollower(Ports.RotarySubsystemMotorFollower, false)),
                    DCMOTOR, MOI, false, CONSTANTS,
                    Optional.of(new AbsoluteEncoderIOCANCoderSim(Ports.RotarySubsystemEncoder,
                        NAME + "Encoder", getCANcoderConfig(true)))));
            case REPLAY:
                return new Rotary(new RotaryMechanism(NAME, CONSTANTS) {});
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }
    }
}
