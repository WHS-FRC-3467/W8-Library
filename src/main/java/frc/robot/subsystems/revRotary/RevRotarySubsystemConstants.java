// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.revRotary;

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
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorIORev;
import frc.lib.io.motor.MotorIORevSim;
import frc.lib.io.motor.MotorIOSim;
import frc.lib.mechanisms.rotary.*;
import frc.lib.mechanisms.rotary.RotaryMechanism.RotaryAxis;
import frc.lib.mechanisms.rotary.RotaryMechanism.RotaryMechCharacteristics;
import frc.robot.Ports;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


/** Add your docs here. */
public class RevRotarySubsystemConstants {
    public static String NAME = "REVRotary";

    public static final Angle TOLERANCE = Degrees.of(2.0);

    public static final AngularVelocity CRUISE_VELOCITY =
        Units.RadiansPerSecond.of(1);
    public static final AngularAcceleration ACCELERATION =
        CRUISE_VELOCITY.div(0.1).per(Units.Second);
    public static final Velocity<AngularAccelerationUnit> JERK = ACCELERATION.per(Second);

    private static final double ROTOR_TO_SENSOR = (2.0 / 1.0);
    private static final double SENSOR_TO_MECHANISM = (2.0 / 1.0);

    public static final Translation3d OFFSET = Translation3d.kZero;

    public static final Angle MIN_ANGLE = Degrees.of(0.0);
    public static final Angle MAX_ANGLE = Rotations.of(.5);
    public static final Angle STARTING_ANGLE = Rotations.of(0.0);
    public static final Distance ARM_LENGTH = Meters.of(1.0);

    public static final RotaryMechCharacteristics CONSTANTS =
        new RotaryMechCharacteristics(OFFSET, ARM_LENGTH, MIN_ANGLE, MAX_ANGLE, STARTING_ANGLE,
            RotaryAxis.PITCH);

    public static final Mass ARM_MASS = Kilograms.of(.01);
    public static final DCMotor DCMOTOR = DCMotor.getKrakenX60(1);
    public static final MomentOfInertia MOI = KilogramSquareMeters
        .of(SingleJointedArmSim.estimateMOI(ARM_LENGTH.in(Meters), ARM_MASS.in(Kilograms)));

    private static final Angle ENCODER_OFFSET = Rotations.of(0.0);

    public static final RevRotarySubsystem.Setpoint DEFAULT_SETPOINT =
        RevRotarySubsystem.Setpoint.STOW;



    // Positional PID
    private static ClosedLoopConfig SLOT0CONFIG = new ClosedLoopConfig()
        .pid(30.0, 0, 0, ClosedLoopSlot.kSlot0);

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
    public static SparkBaseConfig getREVConfig()
    {
        SparkFlexConfig config = new SparkFlexConfig();

        config.voltageCompensation(12.0);
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);
        config.apply(SLOT0CONFIG);

        return config;
    }

    /**
     * Creates the real robot implementation of the rotary mechanism.
     * 
     * <p>
     * This method instantiates the actual hardware objects (TalonFX motors and CANcoder) that will
     * be used when running on a real robot.
     * 
     * @return A RotaryMechanismReal object configured with real hardware
     */
    public static RotaryMechanismReal getReal()
    {
        MotorIO io = new MotorIORev(NAME, Ports.RotarySubsystemMotorMain, true, getREVConfig());

        return new RotaryMechanismReal(io, CONSTANTS, null);
    }

    /**
     * Creates the simulation implementation of the rotary mechanism.
     * 
     * <p>
     * This method creates a physics-based simulation of the mechanism using WPILib's simulation
     * classes. It models the motor, moment of inertia, and other physical properties to provide
     * realistic behavior in simulation.
     * 
     * @return A RotaryMechanismSim object configured for physics simulation
     */
    public static RotaryMechanismSim getSim()
    {
        MotorIOSim io = new MotorIORevSim(
            NAME,
            Ports.RotarySubsystemMotorMain,
            true,
            DCMOTOR,
            getREVConfig());

        return new RotaryMechanismSim(
            io,
            DCMOTOR,
            MOI,
            false,
            CONSTANTS,
            Optional.empty());
    }

    /**
     * Creates the log replay implementation of the rotary mechanism.
     * 
     * <p>
     * This is used with AdvantageKit's log replay feature, which allows you to replay logged data
     * and debug robot code without having the actual robot or running simulation.
     * 
     * @return A RotaryMechanism object for log replay
     */
    public static RotaryMechanism getReplay()
    {
        return new RotaryMechanism(NAME, CONSTANTS) {};
    }
}
