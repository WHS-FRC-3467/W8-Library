// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import frc.lib.actuators.rotary.RotaryActuator;
import frc.lib.actuators.rotary.RotaryActuator.RotaryAxis;
import frc.lib.actuators.rotary.RotaryActuator.RotaryMechCharacteristics;
import frc.lib.actuators.rotary.RotaryActuatorReal;
import frc.lib.actuators.rotary.RotaryActuatorSim;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.io.motor.MotorIOTalonFX;
import frc.lib.io.motor.MotorIOTalonFXSim;
import frc.lib.util.PID;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;
import java.util.Optional;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.MomentOfInertia;

/**
 * Defines configuration and physical constants for the turret hood mechanism, including motion
 * constraints, geometry, motor model, and control gains used to construct the {@link
 * RotaryActuator} instance for different robot modes.
 */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class HoodConstants {
    public static final String NAME = "Hood";

    public static final Angle TOLERANCE = Degrees.of(1.5);

    private static final double GEARING = (168.0 / 10.0) * (28.0 / 11.0);

    // Actual arm angle from horizontal is 15 to 39 deg
    public static final Angle MIN_ANGLE_OFFSET = Degrees.of(15.0);

    public static final Angle MIN_ANGLE = Degrees.of(0.0);
    public static final Angle MAX_ANGLE = Degrees.of(27.0);
    public static final Angle STARTING_ANGLE = Degrees.of(0.0);
    public static final Distance ARM_LENGTH = Inches.of(6.9);

    public static final RotaryMechCharacteristics CONSTANTS =
            new RotaryMechCharacteristics(ARM_LENGTH, MIN_ANGLE, MAX_ANGLE, STARTING_ANGLE, RotaryAxis.PITCH);

    public static final DCMotor DCMOTOR = DCMotor.getKrakenX60(1);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.01);

    private static PID getPID() {
        if (RobotBase.isReal()) {
            return new PID(1000.0, 0.0, 60.0).withS(2.0).withG(12.0);
        } else {
            return new PID(1000.0, 0.0, 80.0);
        }
    }

    // Positional PID
    public static final PID SLOT0_PID = getPID();

    /**
     * Creates a TalonFX motor controller configuration for the hood mechanism. Configures current
     * limits, voltage limits, neutral mode, soft limits, gearing ratios, feedback sensor source,
     * and motion magic parameters.
     *
     * @return configured TalonFXConfiguration for the hood motor
     */
    public static TalonFXConfiguration getFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        if (Robot.isReal()) {
            config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
            config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
        }

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE.in(Units.Rotations);

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE.in(Units.Rotations);

        config.Feedback.SensorToMechanismRatio = GEARING;

        config.Slot0 = Slot0Configs.from(SLOT0_PID.toSlotConfigs())
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        return config;
    }

    /**
     * Creates and configures the hood mechanism based on the current robot mode. Selects the
     * appropriate implementation (real, sim, or replay) and enables tunable PID.
     *
     * @return configured hood mechanism
     */
    public static RotaryActuator<?, ?> get() {
        RotaryActuator<?, ?> mechanism;
        switch (Constants.CURRENT_MODE) {
            case REAL:
                mechanism = new RotaryActuatorReal(
                        NAME, new MotorIOTalonFX(NAME, getFXConfig(), Ports.hood), CONSTANTS, Optional.empty(), "");
                break;
            case SIM:
                mechanism = new RotaryActuatorSim(
                        NAME,
                        new MotorIOTalonFXSim(NAME, MotorType.KrakenX60, DCMOTOR, getFXConfig(), Ports.hood),
                        DCMOTOR,
                        GEARING,
                        1,
                        MOI,
                        false,
                        CONSTANTS,
                        Optional.empty(),
                        "");
                break;
            case REPLAY:
                mechanism = new RotaryActuator<>(NAME, CONSTANTS, new MotorIO() {}, Optional.empty(), "") {};
                break;
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }
        mechanism.enableTunablePID(PIDSlot.SLOT_0, SLOT0_PID);
        return mechanism;
    }
}
