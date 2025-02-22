// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.sim;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.sim.PhysicsSim.SimProfile;
import frc.robot.util.sim.mechanisms.MotionProfiledArmMechanism;

/**
 * Holds information about a simulated Single-Jointed Arm.
 */
public class ArmSimProfile extends SimProfile {

    private final String mName;
    private final TalonFX mTalon;
    private final CANcoder mCANcoder;
    private final MotorSimConfiguration mMotorConst;
    private final ArmSimConfiguration mArmConst;
    private final SingleJointedArmSim mArmSim;
    private final MotionProfiledArmMechanism mArmMech;

    /**
     * Creates a new simulation profile for a Single-Jointed Arm
     * using the WPILib Arm sim.
     * 
     * @param talon
     *      The TalonFX device
     * @param motorConst
     *      Motor Sim configuration values
     * @param armConst
     *      Arm Sim configuration values
     */
    public ArmSimProfile(final String simName,
                        final TalonFX talon,
                        final CANcoder cancoder,
                        final MotorSimConfiguration motorConst,
                        final ArmSimConfiguration armConst) {
        
        this.mName = simName;
        this.mTalon = talon;
        this.mCANcoder = cancoder;
        this.mMotorConst = motorConst;
        this.mArmConst = armConst;

        DCMotor m_armGearbox = mMotorConst.simMotorModelSupplier.get();

        // Create sim object
        this.mArmSim = new SingleJointedArmSim(
            m_armGearbox,
            armConst.kArmReduction,
            SingleJointedArmSim.estimateMOI(armConst.kArmLength, armConst.kArmMass),
            armConst.kArmLength,
            Units.degreesToRadians(armConst.kMinAngleDegrees),
            Units.degreesToRadians(armConst.kMaxAngleDegrees),
            true,
            Units.degreesToRadians(armConst.kDefaultArmSetpointDegrees)
        );

        // Create sim mechanism
        mArmMech = new MotionProfiledArmMechanism(mName);
    }

    /**
     * Runs the simulation profile.
     * 
     * This uses very rudimentary physics simulation and exists to allow users to
     * test features of our products in simulation using our examples out of the
     * box. Users may modify this to utilize more accurate physics simulation.
     */
    public void run() {

        // Get the simulation state for the lead motor
        var simState = mTalon.getSimState();

        // set the supply (battery) voltage for the lead motor simulation state
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Set the input (voltage) to the Arm Simulation
        mArmSim.setInputVoltage(simState.getMotorVoltage());
        // Update the Arm Sim each time throuhgh the loop
        mArmSim.update(getPeriod());

        // Get current position and velocity of the Arm Sim ...
        final double position_rot = Units.radiansToRotations(mArmSim.getAngleRads());
        final double velocity_rps = Units.radiansToRotations(mArmSim.getVelocityRadPerSec());

        // ... and set the position and velocity for the lead motor simulation 
        simState.setRawRotorPosition(position_rot * mArmConst.kArmReduction);
        simState.setRotorVelocity(velocity_rps * mArmConst.kArmReduction);

        // If using an external encoder, update its sim as well
        if (mCANcoder != null) {
            mCANcoder.getSimState().setRawPosition(position_rot * mArmConst.kSensorReduction);
            mCANcoder.getSimState().setVelocity(velocity_rps * mArmConst.kSensorReduction);
        }

        // Update sim mechanism (in degrees)
        mArmMech.update(Units.rotationsToDegrees(position_rot));
    }
}
