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

package frc.robot.subsystems.tower;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.RotationsPerSecond;

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
import frc.lib.io.motor.MotorIOTalonFXSim;
import frc.lib.util.PID;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;

import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.MomentOfInertia;

public class TowerConstants {
    public static String NAME = "Tower";

    public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(40.0);

    private static final double GEARING = (37.0 / 21.0) * (28.0 / 18.0);
    private static final Distance RADIUS = Inches.of(2);

    public static final AngularVelocity TOLERANCE = MAX_VELOCITY.times(0.2);

    private static final DCMotor DCMOTOR = DCMotor.getKrakenX44Foc(1);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.01);

    // Velocity PID
    public static final PID SLOT0_PID = new PID(5.0, 0.0, 0.0).withS(14.0).withV(0.5);

    /**
     * Creates a TalonFX motor controller configuration for the tower mechanism. Configures current
     * limits, voltage limits, neutral mode, gearing ratios, and PID gains.
     *
     * @return configured TalonFXConfiguration for the tower motor
     */
    public static TalonFXConfiguration getFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        if (Robot.isReal()) {
            config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
            config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
        }

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        config.Feedback.RotorToSensorRatio = 1.0;

        config.Feedback.SensorToMechanismRatio = GEARING;

        config.Slot0 = Slot0Configs.from(SLOT0_PID.toSlotConfigs());

        return config;
    }

    /**
     * Creates and configures a Tower subsystem based on the current robot mode. Selects the
     * appropriate flywheel mechanism implementation (real, sim, or replay).
     *
     * @return configured Tower instance
     */
    public static Tower get() {
        FlywheelActuator<?> mechanism;
        switch (Constants.CURRENT_MODE) {
            case REAL:
                mechanism =
                        new FlywheelActuatorReal(
                                NAME, new MotorIOTalonFX(NAME, getFXConfig(), Ports.tower));
                break;
            case SIM:
                mechanism =
                        new FlywheelActuatorSim(
                                NAME,
                                new MotorIOTalonFXSim(NAME, MotorType.KrakenX44, DCMOTOR, getFXConfig(), Ports.tower),
                                DCMOTOR,
                                GEARING,
                                1,
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
        return new Tower(mechanism);
    }
}
