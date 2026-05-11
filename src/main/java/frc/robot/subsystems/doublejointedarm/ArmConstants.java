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
package frc.robot.subsystems.doublejointedarm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;

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

    public static final Angle TOLERANCE = Degrees.of(1.5);

    private static final double GEARING = (168.0 / 10.0) * (28.0 / 11.0);

    // Actual arm angle from horizontal is 15 to 39 deg
    public static final Angle MIN_ANGLE_OFFSET = Degrees.of(15.0);

    public static final Angle MIN_ANGLE = Radians.of(-2000);
    public static final Angle MAX_ANGLE = Degrees.of(1000);
    public static final Angle STARTING_ANGLE = Degrees.of(0.0);
    public static final Distance ARM_LENGTH = Feet.of(6.9);

    public static final ArmJointMechanism.JointCharacteristics LOWER_CONSTANTS =
            new ArmJointMechanism.JointCharacteristics(
                    Feet.of(1), MIN_ANGLE, MAX_ANGLE, STARTING_ANGLE);

    public static final ArmJointMechanism.JointCharacteristics UPPER_CONSTANTS =
            new ArmJointMechanism.JointCharacteristics(
                    Feet.of(1), MIN_ANGLE, MAX_ANGLE, STARTING_ANGLE);

    public static final DCMotor DCMOTOR = DCMotor.getKrakenX60(1);
    public static final MomentOfInertia MOI =
            KilogramSquareMeters.of(SingleJointedArmSim.estimateMOI(1.0, 1.0));

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

        config.Slot0 =
                Slot0Configs.from(SLOT0_PID.toSlotConfigs())
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
                                        NAME, getFXConfig(), Ports.RotarySubsystemMotorFollower),
                                DCMOTOR,
                                MOI,
                                false,
                                UPPER_CONSTANTS,
                                Optional.empty(),
                                ""),
                        new ArmJointMechanismSim(
                                "lower",
                                new MotorIOTalonFXSim(
                                        NAME, getFXConfig(), Ports.RotarySubsystemEncoder),
                                DCMOTOR,
                                MOI,
                                false,
                                LOWER_CONSTANTS,
                                Optional.empty(),
                                ""),
                        NAME);

        return new Arm(mech);
    }
}
