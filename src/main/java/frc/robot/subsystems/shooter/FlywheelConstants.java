// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import frc.lib.actuators.flywheel.FlywheelActuator;
import frc.lib.actuators.flywheel.FlywheelActuatorReal;
import frc.lib.actuators.flywheel.FlywheelActuatorSim;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.io.motor.MotorIOTalonFX;
import frc.lib.io.motor.MotorIOTalonFX.TalonFXFollower;
import frc.lib.io.motor.MotorIOTalonFXSim;
import frc.lib.util.PID;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.MomentOfInertia;

/** Constants and factory methods for the shooter flywheel. */
public class FlywheelConstants {

    public static final String NAME = "Flywheel";

    public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(69.0);
    public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(30.0);
    public static final AngularAcceleration BROWNOUT_MAX_ACCELERATION = RotationsPerSecondPerSecond.of(15.0);

    private static final double GEARING = (32.0 / 24.0);

    public static final AngularVelocity TOLERANCE = RotationsPerSecond.of(1.0);

    private static final DCMotor DCMOTOR = DCMotor.getKrakenX60(4);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.001);

    public static final Distance FLYWHEEL_RADIUS = Inches.of(4.0);

    private static PID getPID() {
        if (RobotBase.isReal()) {
            return new PID(16.0, 0.0, 0.0).withS(5.5).withA(0.8); // Flywheel tunings
            // return new PID(20.0, 0.0, 0.0).withS(3).withA(0.47).withV(.11); // No flywheel tuning
        } else {
            return new PID(10.0, 0.0, 0.0).withV(1.8);
        }
    }

    // Velocity PID
    public static final PID SLOT0_PID = getPID();

    /**
     * Creates a TalonFX motor controller configuration for the flywheel mechanism. Configures
     * current limits, voltage limits, neutral mode, gearing ratios, and PID gains.
     *
     * @return configured TalonFXConfiguration for the flywheel motor
     */
    public static TalonFXConfiguration getFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        config.Feedback.RotorToSensorRatio = 1.0;

        config.Feedback.SensorToMechanismRatio = GEARING;

        if (Robot.isReal()) {
            config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
            config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
        }

        config.Slot0 = Slot0Configs.from(SLOT0_PID.toSlotConfigs());
        config.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY.in(RotationsPerSecond);
        config.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION.in(RotationsPerSecondPerSecond);

        return config;
    }

    /**
     * Creates and configures the flywheel mechanism based on the current robot mode. Selects the
     * appropriate implementation (real, sim, or replay) and enables tunable PID.
     *
     * @return configured flywheel mechanism
     */
    public static FlywheelActuator<?> get() {
        FlywheelActuator<?> mechanism;
        switch (Constants.CURRENT_MODE) {
            case REAL:
                mechanism = new FlywheelActuatorReal(
                        NAME,
                        new MotorIOTalonFX(
                                NAME,
                                getFXConfig(),
                                Ports.topLeftFlywheel,
                                new TalonFXFollower(Ports.topRightFlywheel, true),
                                new TalonFXFollower(Ports.bottomLeftFlywheel, false),
                                new TalonFXFollower(Ports.bottomRightFlywheel, true)));

                break;
            case SIM:
                mechanism = new FlywheelActuatorSim(
                        NAME,
                        new MotorIOTalonFXSim(
                                NAME,
                                MotorType.KrakenX60,
                                DCMOTOR,
                                getFXConfig(),
                                Ports.topLeftFlywheel,
                                new TalonFXFollower(Ports.topRightFlywheel, true),
                                new TalonFXFollower(Ports.bottomLeftFlywheel, false),
                                new TalonFXFollower(Ports.bottomRightFlywheel, true)),
                        DCMOTOR,
                        GEARING,
                        4,
                        MOI,
                        TOLERANCE);
                break;
            case REPLAY:
                mechanism = new FlywheelActuator<>(NAME, new MotorIO() {}) {};
                break;
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }
        mechanism.enableTunablePID(PIDSlot.SLOT_0, SLOT0_PID);
        mechanism.withRadius(FLYWHEEL_RADIUS);
        return mechanism;
    }
}
