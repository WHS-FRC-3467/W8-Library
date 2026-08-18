/*
 * Copyright (C) 2025 Windham Windup
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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.util.BatterySimCurrentAccumulator;

/**
 * Physics sim implementation of module IO. The sim models are configured using
 * a set of module onstants from Phoenix. Simulation is always based on voltage 
 * control.
 */
public class ModuleIOSim implements ModuleIO {
    // TunerConstants doesn't support separate sim constants, so they are declared
    // locally
    private static final double DRIVE_KP = 0.05;
    private static final double DRIVE_KD = 0.0;
    private static final double DRIVE_KS = 0.0;
    private static final double TURN_KP = 8.0;
    private static final double TURN_KD = 0.0;
    /** 
    * Bare, unreduced DC motor models for the drive and turn motor simulations (qty 1 ea). 
    * The gear ratio is not applied to these models; it is instead applied in the 
    * {@link LinearSystemId#createDCMotorSystem(DCMotor, double, double)} call. 
    */
    private static final DCMotor DRIVE_MOTOR_MODEL = DCMotor.getKrakenX60Foc(1);
    /** See {@link #DRIVE_MOTOR_MODEL}. */
    private static final DCMotor TURN_MOTOR_MODEL = DCMotor.getKrakenX44Foc(1);

    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;
    // Feedforward V per (rad/s) of wheel shaft; derived from motor Kv and gear
    // ratio so the simulation converges to the exact commanded velocity with no
    // steady-state error.
    private final double driveKv;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
    private PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);
    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public ModuleIOSim(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        // Create drive and turn sim models
        driveSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DRIVE_MOTOR_MODEL, constants.DriveInertia, constants.DriveMotorGearRatio),
                DRIVE_MOTOR_MODEL);
        turnSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        TURN_MOTOR_MODEL, constants.SteerInertia, constants.SteerMotorGearRatio),
                TURN_MOTOR_MODEL);

        // Compute drive FF from the actual motor model: V/(rad/s at wheel) = gearRatio
        // / Kv_motor
        // This eliminates steady-state velocity error in the simulation.
        driveKv = constants.DriveMotorGearRatio / DRIVE_MOTOR_MODEL.KvRadPerSecPerVolt;

        // Enable wrapping for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts = driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
        } else {
            driveController.reset();
        }
        if (turnClosedLoop) {
            turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
        } else {
            turnController.reset();
        }

        // Update simulation state
        // Apply controller-commanded clamped voltage to the sim models; values are internally clamped to 
        // be within the currently available loaded battery supply voltage
        double supplyVoltageVolts = RobotController.getBatteryVoltage();
        driveSim.setInputVoltage(driveAppliedVolts);
        turnSim.setInputVoltage(turnAppliedVolts);
        driveSim.update(0.02);
        turnSim.update(0.02);
        // Update the battery load accumulator with the current draw from both motors
        Current driveSupplyCurrent = Amps.of(Math.abs(driveSim.getCurrentDrawAmps()));
        Current turnSupplyCurrent = Amps.of(Math.abs(turnSim.getCurrentDrawAmps()));
        BatterySimCurrentAccumulator.addCurrentLoad(driveSupplyCurrent.plus(turnSupplyCurrent));

        // Update drive inputs
        // Note: drive position, velocity, and torque are reported in mechanism units 
        inputs.driveConnected = true;
        inputs.drivePositionRad = driveSim.getAngularPositionRad(); 
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec(); 
        inputs.driveSupplyVoltageVolts = supplyVoltageVolts;
        inputs.driveAppliedVoltageVolts = driveSim.getInputVoltage(); 
        inputs.driveSupplyCurrentAmps = driveSupplyCurrent.in(Amps);
        inputs.driveTorqueCurrentAmps = Math.abs(driveSim.getTorqueNewtonMeters() / (driveSim.getGearing() * DRIVE_MOTOR_MODEL.KtNMPerAmp)); 

        // Update turn inputs
        // Note: turn position, velocity, and torque are reported in mechanism units
        inputs.turnConnected = true;
        inputs.turnEncoderConnected = true;
        double turnPositionRad = turnSim.getAngularPositionRad();
        inputs.turnAbsolutePosition = new Rotation2d(turnPositionRad); 
        inputs.turnPosition = new Rotation2d(turnPositionRad); 
        inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec(); 
        inputs.turnSupplyVoltageVolts = supplyVoltageVolts;
        inputs.turnAppliedVoltageVolts = turnSim.getInputVoltage(); 
        inputs.turnSupplyCurrentAmps = turnSupplyCurrent.in(Amps);
        inputs.turnTorqueCurrentAmps = Math.abs(turnSim.getTorqueNewtonMeters() / (turnSim.getGearing() * TURN_MOTOR_MODEL.KtNMPerAmp)); 


        // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
        // matter)
        inputs.odometryTimestamps = new double[] { Timer.getFPGATimestamp() };
        inputs.odometryDrivePositionsRad = new double[] { inputs.drivePositionRad };
        inputs.odometryTurnPositions = new Rotation2d[] { inputs.turnPosition };
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveClosedLoop = false;
        driveAppliedVolts = output;
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnClosedLoop = false;
        turnAppliedVolts = output;
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        driveClosedLoop = true;
        driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;
        driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnClosedLoop = true;
        turnController.setSetpoint(rotation.getRadians());
    }
}
