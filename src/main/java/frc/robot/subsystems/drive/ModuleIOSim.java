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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.mechanisms.MotorSimCalcs;
import frc.lib.util.BatteryModel;

/**
 * Physics sim implementation of module IO. The sim models are configured using
 * a set of module constants from Phoenix. Simulation is always based on voltage 
 * control.
 */
public class ModuleIOSim implements ModuleIO {
    private final String name;

    // TunerConstants doesn't support separate sim constants, so they are declared
    // locally
    private static final double DRIVE_KP = 0.05;
    private static final double DRIVE_KD = 0.0;
    private static final double DRIVE_KS = 0.0;
    private static final double TURN_KP = 8.0;
    private static final double TURN_KD = 0.0;

    /** 
    * Bare, unreduced DC motor models for the drive and turn motor simulations (qty 1 ea). 
    * The gear ratios are not applied to these objects; they are instead applied in the 
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
    private final PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
    private final PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);
    private double driveFFVolts = 0.0;

    // Electrical
    private double driveUnclampedAppliedVolts = 0.0;
    private double turnUnclampedAppliedVolts = 0.0;

    // Simulation time-stepping parameters
    private static final double SIMULATION_STEP_SECONDS = 0.02;
    private static final int SIM_SUBSTEPS = 6;

    // Boolean indicating whether regenerative braking is feeding the bus. Can vary from module to module.
    public boolean isDriveRegenerating = false;
    public boolean isTurnRegenerating = false;

    // Average of the drive and turn supply currents over a simulation step for use in battery load modeling
    private double driveSupplyCurrentAverageAmps = 0.0;
    private double turnSupplyCurrentAverageAmps = 0.0;

    /**
     * Constructs a new ModuleIOSim instance.
     *
     * @param name Name of the module for logging purposes
     * @param constants Module-specific constants for configuring hardware devices
     */
    public ModuleIOSim(String name,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.name = name;

        /* 
         * Create Drive and Turn sim models. 
         * 
         * DCMotorSim extends LinearSystemSim and together, given a LinearSystem, provide a state space 
         * model and associated API for a DC motor gearbox which produces mechanism-space positions, velocities, 
         * accelerations, and torques. LinearSystem provides the state space computation machinery, LinearSystemSim 
         * provides the update functionality / basic API, and DCMotorSim provides the final simulation API and uses 
         * the results from the state space model in some additional modeling helpers, including electrical modeling 
         * through the DCMotor class.
         * 
         * Along with mechanism / output load MOI, the system factory expects either:
         * 
         * A) bare motor model if actual gearing is passed, or
         * B) pre-reduced motor model with 1.0 passed into gearing. 
         */
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
        /* Run closed-loop control */
        if (driveClosedLoop) {
            driveUnclampedAppliedVolts = driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
        } else {
            driveController.reset();
        }
        if (turnClosedLoop) {
            turnUnclampedAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
        } else {
            turnController.reset();
        }

        /*  
         * Apply controller-commanded voltage to the sim models. The control request is clamped to adhere to current 
         * limits and available battery voltage.
         */
        double supplyVoltageVolts = RobotController.getBatteryVoltage();
        /*
         * Continue to simulate the robot controller as writing/reading at SIMULATION_STEP_SECONDS (e.g. 20 ms, 50 Hz), but current 
         * limit faster by a factor of SIM_SUBSTEPS. This results in more precise current limiting. Note that actual motor controllers 
         * often limit at ~ 1000 Hz.
         * 
         * Over 0.02 s, wheel speed tends to change drastically; current limiting off of 20 ms stale back-EMF often 
         * results in logged stator currents that are notably below established current limits.
         */
        driveSupplyCurrentAverageAmps = 0.0;
        turnSupplyCurrentAverageAmps = 0.0;

        for (int i = 0; i < SIM_SUBSTEPS; i++) { 
            double driveClampedAppliedVolts = MotorSimCalcs.clampVoltageToTorqueCurrentLimit(
                driveUnclampedAppliedVolts, 
                driveSim.getAngularVelocityRadPerSec(),
                driveSim.getGearing(), 
                DRIVE_MOTOR_MODEL, 
                DriveConstants.getKSlipCurrent().in(Amps),
                supplyVoltageVolts);
            driveSim.setInputVoltage(driveClampedAppliedVolts);

            double turnClampedAppliedVolts = MotorSimCalcs.clampVoltageToTorqueCurrentLimit(
                turnUnclampedAppliedVolts, 
                turnSim.getAngularVelocityRadPerSec(),
                turnSim.getGearing(), 
                TURN_MOTOR_MODEL, 
                DriveConstants.getTurnCurrentMax().in(Amps),
                supplyVoltageVolts);
            turnSim.setInputVoltage(turnClampedAppliedVolts);

            // Substep the sim
            driveSim.update(SIMULATION_STEP_SECONDS / SIM_SUBSTEPS);
            turnSim.update(SIMULATION_STEP_SECONDS / SIM_SUBSTEPS);

            // Average supply currents over the substeps for more accurate battery load modeling
            driveSupplyCurrentAverageAmps += MotorSimCalcs.getSupplyCurrentAmps(supplyVoltageVolts, driveClampedAppliedVolts, 
            MotorSimCalcs.getTorqueCurrentAmps(driveSim, DRIVE_MOTOR_MODEL.KtNMPerAmp));
            turnSupplyCurrentAverageAmps += MotorSimCalcs.getSupplyCurrentAmps(supplyVoltageVolts, turnClampedAppliedVolts, 
            MotorSimCalcs.getTorqueCurrentAmps(turnSim, TURN_MOTOR_MODEL.KtNMPerAmp));
            if (i == SIM_SUBSTEPS - 1) {
                driveSupplyCurrentAverageAmps /= SIM_SUBSTEPS;
                turnSupplyCurrentAverageAmps /= SIM_SUBSTEPS;
            }
        }
  
        /* Update the battery load accumulator with the current draw from both motors. 
         * This new load will be applied next loop. */
        BatteryModel.addCurrentLoad(driveSupplyCurrentAverageAmps);
        BatteryModel.addCurrentLoad(turnSupplyCurrentAverageAmps); 

        /* Write the motor inputs for this control loop; these are the values from the most recent sim substep. */
        // Clamped applied volts
        double driveAppliedVoltageVolts = driveSim.getInputVoltage(); 
        // Can equivalently calculate torque current via getCurrent(speed, voltage)
        double driveTorqueCurrentAmps = MotorSimCalcs.getTorqueCurrentAmps(driveSim, DRIVE_MOTOR_MODEL.KtNMPerAmp);
        double driveSupplyCurrentAmps = MotorSimCalcs.getSupplyCurrentAmps(supplyVoltageVolts, driveAppliedVoltageVolts, driveTorqueCurrentAmps);
        // A positive supply current is a bus draw and a negative supply current is a bus regen
        isDriveRegenerating = driveSupplyCurrentAmps < 0;
        

        // Clamped applied volts
        double turnAppliedVoltageVolts = turnSim.getInputVoltage();
        // Can equivalently calculate torque current via getCurrent(speed, voltage)
        double turnTorqueCurrentAmps = MotorSimCalcs.getTorqueCurrentAmps(turnSim, TURN_MOTOR_MODEL.KtNMPerAmp);
        double turnSupplyCurrentAmps = MotorSimCalcs.getSupplyCurrentAmps(supplyVoltageVolts, turnAppliedVoltageVolts, turnTorqueCurrentAmps);
        // A positive supply current is a bus draw and a negative supply current is a bus regen
        isTurnRegenerating = turnSupplyCurrentAmps < 0;
        
        // Update Drive inputs
        // Note: Drive position, velocity, and torque are reported in mechanism units 
        inputs.driveConnected = true;
        inputs.drivePositionRad = driveSim.getAngularPositionRad(); 
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec(); 
        inputs.driveSupplyVoltageVolts = supplyVoltageVolts;
        inputs.driveAppliedVoltageVolts = driveAppliedVoltageVolts;
        inputs.driveSupplyCurrentAmps = driveSupplyCurrentAmps;
        inputs.driveTorqueCurrentAmps = driveTorqueCurrentAmps;

        // Update Turn inputs
        // Note: Turn position, velocity, and torque are reported in mechanism units
        inputs.turnConnected = true;
        inputs.turnEncoderConnected = true;
        double turnPositionRad = turnSim.getAngularPositionRad();
        inputs.turnAbsolutePosition = new Rotation2d(turnPositionRad); 
        inputs.turnPosition = new Rotation2d(turnPositionRad); 
        inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec(); 
        inputs.turnSupplyVoltageVolts = supplyVoltageVolts;
        inputs.turnAppliedVoltageVolts = turnAppliedVoltageVolts;
        inputs.turnSupplyCurrentAmps = turnSupplyCurrentAmps;
        inputs.turnTorqueCurrentAmps = turnTorqueCurrentAmps;

        // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
        // matter)
        inputs.odometryTimestamps = new double[] { Timer.getFPGATimestamp() };
        inputs.odometryDrivePositionsRad = new double[] { inputs.drivePositionRad };
        inputs.odometryTurnPositions = new Rotation2d[] { inputs.turnPosition };

        // Log module-specific simulation parameters for diagnostics
        MotorSimCalcs.logSimState("Sim/Modules/Drive/" + name + "/", 
            supplyVoltageVolts, driveUnclampedAppliedVolts, driveAppliedVoltageVolts, 
            DriveConstants.getKSlipCurrent().in(Amps), driveTorqueCurrentAmps, driveSupplyCurrentAmps, 
            driveSupplyCurrentAverageAmps, isDriveRegenerating);
        MotorSimCalcs.logSimState("Sim/Modules/Turn/" + name + "/", 
            supplyVoltageVolts, turnUnclampedAppliedVolts, turnAppliedVoltageVolts, 
            DriveConstants.getTurnCurrentMax().in(Amps), turnTorqueCurrentAmps, turnSupplyCurrentAmps, 
            turnSupplyCurrentAverageAmps, isTurnRegenerating);
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveClosedLoop = false;
        driveUnclampedAppliedVolts = output;
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnClosedLoop = false;
        turnUnclampedAppliedVolts = output;
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

    @Override
    public String getName() {
        return name;
    }
}
