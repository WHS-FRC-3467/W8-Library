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
import frc.robot.Constants;
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
public class RevRotaryConstants {
    public static String NAME = "REVRotary";

    public static final Angle TOLERANCE = Degrees.of(2.0);

    public static final AngularVelocity CRUISE_VELOCITY =
        Units.RadiansPerSecond.of(1);
    public static final AngularAcceleration ACCELERATION =
        CRUISE_VELOCITY.div(0.1).per(Units.Second);
    public static final Velocity<AngularAccelerationUnit> JERK = ACCELERATION.per(Second);

    private static final double ROTOR_TO_SENSOR = (2.0 / 1.0);
    private static final double SENSOR_TO_MECHANISM = (2.0 / 1.0);
    private static final double GEAR_RATIO = ROTOR_TO_SENSOR * SENSOR_TO_MECHANISM;

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

    public static final RevRotary.Setpoint DEFAULT_SETPOINT =
        RevRotary.Setpoint.STOW;



    // Positional PID
    private static ClosedLoopConfig SLOT0CONFIG = new ClosedLoopConfig()
        .pid(30.0, 0, 5.0, ClosedLoopSlot.kSlot0); // Added D gain to match TalonFX config

    /**
     * Creates and returns the SparkFlex/SparkMax motor controller configuration for the rotary mechanism.
     * 
     * <p>
     * This configuration includes:
     * <ul>
     * <li>Voltage compensation for consistent power output</li>
     * <li>Idle (brake) mode to hold position when not moving</li>
     * <li>Motor inversion setting</li>
     * <li>Encoder position and velocity conversion factors for proper feedback scaling</li>
     * <li>PID gains applied to the selected closed-loop slot</li>
     * <!-- Add other REV-specific features as needed -->
     * </ul>
     * 
     * @return A configured {@link SparkBaseConfig} object ready to apply to a REV Robotics SparkFlex or SparkMax motor controller
     */
    public static SparkBaseConfig getREVConfig()
    {
        SparkFlexConfig config = new SparkFlexConfig();

        config.voltageCompensation(12.0);
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);

        // Add gear ratio configuration for position/velocity conversion
        config.encoder.positionConversionFactor(1.0 / GEAR_RATIO);
        config.encoder.velocityConversionFactor(1.0 / GEAR_RATIO / 60.0); // RPM to RPS

        // Add soft limits to match TalonFX behavior

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
    public static RevRotary getReal()
    {
        MotorIO io = new MotorIORev(NAME, Ports.revRotarySubsytemMotorMain, true, getREVConfig());

        return new RevRotary(new RotaryMechanismReal(io, CONSTANTS, Optional.empty()));
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
    public static RevRotary getSim()
    {
        MotorIOSim io = new MotorIORevSim(
            NAME,
            Ports.revRotarySubsytemMotorMain,
            true,
            DCMOTOR,
            getREVConfig());

        return new RevRotary(new RotaryMechanismSim(
            io,
            DCMOTOR,
            MOI,
            false,
            CONSTANTS,
            Optional.empty()));
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
    public static RevRotary getReplay()
    {
        return new RevRotary(new RotaryMechanism(NAME, CONSTANTS) {});
    }

    /**
     * Method to get the appropriate RotaryMechanism based on the current robot mode.
     * 
     * @return RotaryMechanism instance for the current mode (real, sim, or replay)
     */
    public static RevRotary get()
    {
        switch (Constants.currentMode) {
            case REAL:
                return getReal();
            case SIM:
                return getSim();
            case REPLAY:
                return getReplay();
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }
    }
}
