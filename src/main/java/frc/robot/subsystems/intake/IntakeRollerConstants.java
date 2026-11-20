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

package frc.robot.subsystems.intake;

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
import frc.lib.io.motor.MotorIOTalonFXSim;
import frc.lib.util.PID;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.MomentOfInertia;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class IntakeRollerConstants {

    public static final String NAME = "Intake Roller";

    public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(35.0);
    public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(110.0);

    private static final double GEARING = (22.0 / 12.0) * (24.0 / 18.0);

    public static final AngularVelocity TOLERANCE = RotationsPerSecond.of(5.0);

    private static final DCMotor DCMOTOR = DCMotor.getKrakenX60Foc(1);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.1);

    // Velocity PID
    public static final PID SLOT0_PID = new PID(8.0, 0.0, 0.0).withS(5.0);

    /**
     * Creates and configures a TalonFX motor controller configuration for the intake roller.
     *
     * @return The configured TalonFX configuration
     */
    public static TalonFXConfiguration getFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        if (Robot.isReal()) {
            config.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
            config.TorqueCurrent.PeakReverseTorqueCurrent = -120.0;
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
        config.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY.in(RotationsPerSecond);
        config.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION.in(RotationsPerSecondPerSecond);

        return config;
    }

    /**
     * Factory method to create a FlywheelActuator instance for the intake roller subsystem. Creates
     * the appropriate mechanism based on the current robot mode (REAL, SIM, or REPLAY).
     *
     * @return A configured FlywheelActuator instance
     */
    public static FlywheelActuator<?> getMechanism() {
        FlywheelActuator<?> mechanism;
        switch (Constants.CURRENT_MODE) {
            case REAL:
                mechanism = new FlywheelActuatorReal(NAME, new MotorIOTalonFX(NAME, getFXConfig(), Ports.intakeRoller));
                break;
            case SIM:
                mechanism = new FlywheelActuatorSim(
                        NAME,
                        new MotorIOTalonFXSim(NAME, MotorType.KrakenX60, DCMOTOR, getFXConfig(), Ports.intakeRoller),
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
        return mechanism;
    }
}
