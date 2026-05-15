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
package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import frc.lib.io.motor.MotorIOTalonFXSim;
import frc.lib.mechanisms.doublejointedarm.ArmJointMechanism;
import frc.lib.mechanisms.doublejointedarm.ArmJointMechanismSim;
import frc.lib.mechanisms.doublejointedarm.DoubleJointedArmMechanismSim;
import frc.lib.mechanisms.rotary.*;
import frc.lib.util.PID;
import frc.robot.Ports;
import frc.robot.Robot;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;

import java.util.Optional;

/**
 * Defines configuration and physical constants for the turret hood mechanism, including motion
 * constraints, geometry, motor model, and control gains used to construct the {@link
 * RotaryMechanism} instance for different robot modes.
 */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class ArmConstants {
    public static final String NAME = "Arm";

    private static final double GEARING = 10;

    public static final Angle MIN_ANGLE = Degrees.of(-1800);
    public static final Angle MAX_ANGLE = Degrees.of(1800);
    public static final Angle STARTING_ANGLE = Degrees.of(90.0);
    public static final Distance ARM_LENGTH = Feet.of(1.0);

    public static final ArmJointMechanism.JointCharacteristics LOWER_CONSTANTS =
            new ArmJointMechanism.JointCharacteristics(
                    ARM_LENGTH, MIN_ANGLE, MAX_ANGLE, Degrees.of(0.0));

    public static final ArmJointMechanism.JointCharacteristics UPPER_CONSTANTS =
            new ArmJointMechanism.JointCharacteristics(
                    ARM_LENGTH, MIN_ANGLE, MAX_ANGLE, Degrees.of(0.0));

    public static final DCMotor DCMOTOR = DCMotor.getKrakenX60(1);
    public static final MomentOfInertia MOI =
            KilogramSquareMeters.of(SingleJointedArmSim.estimateMOI(ARM_LENGTH.in(Meters), 1.0));
    public static final MomentOfInertia MOI_LOWER =
            KilogramSquareMeters.of(SingleJointedArmSim.estimateMOI(ARM_LENGTH.in(Meters), 2.0));
    private static final double kG = 20;

    private static PID getPID() {
        if (RobotBase.isReal()) {
            return new PID(1000.0, 0.0, 60.0).withS(2.0).withG(kG);
        } else {
            return new PID(100.0, 0.0, 80.0);
        }
    }

    private static PID getPID2() {
        if (RobotBase.isReal()) {
            return new PID(1000.0, 0.0, 60.0).withS(2.0).withG(kG);
        } else {
            return new PID(100.0, 0.0, 80.0).withG(20.7);
        }
    }

    // Positional PID
    public static final PID SLOT0_PID = getPID();
    public static final PID SLOT0_PID2 = getPID2();

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
            config.TorqueCurrent.PeakForwardTorqueCurrent = 800.0;
            config.TorqueCurrent.PeakReverseTorqueCurrent = -800.0;
        }

        config.Voltage.PeakForwardVoltage = 24.0;
        config.Voltage.PeakReverseVoltage = -24.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE.in(Units.Rotations);

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE.in(Units.Rotations);

        config.Feedback.SensorToMechanismRatio = GEARING;

        config.Feedback.RotorToSensorRatio = GEARING;

        config.Slot0 =
                Slot0Configs.from(SLOT0_PID.toSlotConfigs())
                        .withGravityType(GravityTypeValue.Arm_Cosine)
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        return config;
    }

    /**
     * Creates a TalonFX motor controller configuration for the hood mechanism. Configures current
     * limits, voltage limits, neutral mode, soft limits, gearing ratios, feedback sensor source,
     * and motion magic parameters.
     *
     * @return configured TalonFXConfiguration for the hood motor
     */
    public static TalonFXConfiguration getFXConfig2() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        if (Robot.isReal()) {
            config.TorqueCurrent.PeakForwardTorqueCurrent = 800.0;
            config.TorqueCurrent.PeakReverseTorqueCurrent = -800.0;
        }

        config.Voltage.PeakForwardVoltage = 24.0;
        config.Voltage.PeakReverseVoltage = -24.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE.in(Units.Rotations);

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE.in(Units.Rotations);

        config.Feedback.SensorToMechanismRatio = GEARING;

        config.Slot0 =
                Slot0Configs.from(SLOT0_PID2.toSlotConfigs())
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
    public static Arm get() {
        var mech =
                new DoubleJointedArmMechanismSim(
                        new ArmJointMechanismSim(
                                "upper",
                                new MotorIOTalonFXSim(
                                        NAME + "upper",
                                        getFXConfig(),
                                        Ports.RotarySubsystemMotorFollower),
                                DCMOTOR,
                                MOI,
                                true,
                                UPPER_CONSTANTS,
                                Optional.empty(),
                                ""),
                        new ArmJointMechanismSim(
                                "lower",
                                new MotorIOTalonFXSim(
                                        NAME + "lower",
                                        getFXConfig2(),
                                        Ports.RotarySubsystemEncoder),
                                DCMOTOR,
                                MOI_LOWER,
                                true,
                                LOWER_CONSTANTS,
                                Optional.empty(),
                                ""),
                        NAME);

        return new Arm(mech);
    }
}
