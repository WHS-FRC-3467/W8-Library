// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rotary;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Meters;
import com.ctre.phoenix6.configs.*;
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
import frc.lib.io.motor.MotorIOTalonFX;
import frc.lib.io.motor.MotorIOTalonFX.TalonFXFollower;
import frc.lib.io.motor.MotorIOTalonFXSim;
import frc.lib.mechanisms.rotary.*;
import frc.lib.mechanisms.rotary.RotaryMechanism.RotaryMechCharacteristics;
import frc.robot.Ports;

/** Add your docs here. */
public class RotarySubsystemConstants {
    public static String NAME = "Rotary";

    public static final Angle TOLERANCE = Degrees.of(2.0);

    public static final AngularVelocity CRUISE_VELOCITY = Units.RadiansPerSecond.of(2 * Math.PI);
    public static final AngularAcceleration ACCELERATION =
        CRUISE_VELOCITY.div(0.1).per(Units.Second);
    public static final Velocity<AngularAccelerationUnit> JERK = ACCELERATION.per(Second);

    public static final double GEARING = (2.0 / 1.0);

    public static final Translation3d OFFSET = Translation3d.kZero;

    public static final Angle MIN_ANGLE = Degrees.of(0.0);
    public static final Angle MAX_ANGLE = Degrees.of(90.0);
    public static final Angle STARTING_ANGLE = Rotations.of(0.5);
    public static final Distance ARM_LENGTH = Meters.of(1.0);

    public static final RotaryMechCharacteristics CONSTANTS =
        new RotaryMechCharacteristics(OFFSET, ARM_LENGTH, MIN_ANGLE, MAX_ANGLE, STARTING_ANGLE);

    public static final Mass ARM_MASS = Kilograms.of(.01);
    public static final DCMotor DCMOTOR = DCMotor.getKrakenX60(1);
    public static final MomentOfInertia MOI = KilogramSquareMeters
        .of(SingleJointedArmSim.estimateMOI(ARM_LENGTH.in(Meters), ARM_MASS.in(Kilograms)));

    public static final RotarySubsystem.Setpoint DEFAULT_SETPOINT = RotarySubsystem.Setpoint.STOW;



    // Positional PID
    private static Slot0Configs SLOT0CONFIG = new Slot0Configs()
        .withKP(30.0)
        .withKI(0.0)
        .withKD(0.0);

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

        config.Feedback.RotorToSensorRatio = 1.0;

        config.Feedback.SensorToMechanismRatio = GEARING;

        config.Slot0 = SLOT0CONFIG;

        return config;
    }

    public static RotaryMechanismReal getReal()
    {
        return new RotaryMechanismReal(
            new MotorIOTalonFX(NAME, getFXConfig(), Ports.RotarySubsystemMotorMain,
                new TalonFXFollower(Ports.RotarySubsystemMotorFollower, false)),
            CONSTANTS);
    }

    public static RotaryMechanismSim getSim()
    {
        return new RotaryMechanismSim(
            new MotorIOTalonFXSim(NAME, getFXConfig(), Ports.RotarySubsystemMotorMain,
                new TalonFXFollower(Ports.RotarySubsystemMotorFollower, false)),
            DCMOTOR, MOI, false, CONSTANTS);
    }

    public static RotaryMechanism getReplay()
    {
        return new RotaryMechanism(NAME, CONSTANTS) {};
    }
}
