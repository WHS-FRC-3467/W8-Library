// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static org.wpilib.units.Units.Amps;
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

import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.MomentOfInertia;

/** Add your docs here. */
public class IndexerConstants {

    public static final String NAME = "Indexer";

    public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(45.0);
    public static final AngularAcceleration MAX_ACCELERATION =
            RotationsPerSecondPerSecond.of(193.0);

    private static final double GEARING = (42.0 / 26.0);

    public static final Distance RADIUS = Inches.of(1.25);

    public static final AngularVelocity TOLERANCE = RotationsPerSecond.of(5.0);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(30.0);
    public static final Current BROWNOUT_SUPPLY_CURRENT_LIMIT = Amps.of(20.0);

    private static final DCMotor DCMOTOR = DCMotor.getKrakenX44Foc(1);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.001);

    // Velocity PID
    public static final PID SLOT0_PID = new PID(5.0, 0.0, 0.0).withS(5.0);

    /**
     * Creates and configures a TalonFX motor controller configuration for the indexer.
     *
     * @return The configured TalonFX configuration
     */
    public static TalonFXConfiguration getFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT.in(Amps);
        config.CurrentLimits.SupplyCurrentLowerLimit = SUPPLY_CURRENT_LIMIT.in(Amps);
        config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        if (Robot.isReal()) {
            config.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
            config.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;
        }

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        config.Feedback.RotorToSensorRatio = 1.0;

        config.Feedback.SensorToMechanismRatio = GEARING;

        config.Slot0 = Slot0Configs.from(SLOT0_PID.toSlotConfigs());

        return config;
    }

    /**
     * Factory method to create an Indexer subsystem instance. Creates the appropriate mechanism
     * based on the current robot mode (REAL, SIM, or REPLAY).
     *
     * @return A fully configured Indexer subsystem
     */
    private static FlywheelActuator<?> getMechanism() {
        FlywheelActuator<?> mechanism;
        switch (Constants.CURRENT_MODE) {
            case REAL:
                mechanism =
                        new FlywheelActuatorReal(
                                NAME,
                                new MotorIOTalonFX(
                                        NAME,
                                        getFXConfig(),
                                        Ports.indexer,
                                        new TalonFXFollower(Ports.indexerFollower, false)));
                break;
            case SIM:
                mechanism =
                        new FlywheelActuatorSim(
                                NAME,
                                new MotorIOTalonFXSim(
                                        NAME,
                                        MotorType.KrakenX44,
                                        DCMOTOR,
                                        getFXConfig(),
                                        Ports.indexer,
                                        new TalonFXFollower(Ports.indexerFollower, false)),
                                DCMOTOR,
                                GEARING,
                                2,
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
        mechanism.withRadius(RADIUS);
        return mechanism;
    }

    /**
     * Creates and configures a complete Indexer subsystem. Combines floor and centering flywheel
     * mechanisms into a unified indexer subsystem.
     *
     * @return configured Indexer instance
     */
    public static Indexer get() {
        return new Indexer(getMechanism());
    }
}
