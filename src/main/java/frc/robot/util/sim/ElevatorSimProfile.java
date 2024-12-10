// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.sim;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.sim.PhysicsSim.SimProfile;
import frc.robot.util.sim.mechanisms.MotionProfiledElevatorMechanism;

/**
 * Holds information about a simulated Elevatore.
 */
public class ElevatorSimProfile extends SimProfile {

    private final String mName;
    private final TalonFX mTalon;
    private final CANcoder mCANcoder;
    private final MotorSimConfiguration mMotorConst;
    private final ElevatorSimConfiguration mElevConst;
    private final ElevatorSim mElevatorSim;
    private final MotionProfiledElevatorMechanism mElevatorMech;

    /**
     * Creates a new simulation profile for an Elevator
     * using the WPILib Elevator sim.
     * 
     * @param talon
     *      The TalonFX device
     * @param motorConst
     *      Motor Sim configuration values
     * @param armConst
     *      Arm Sim configuration values
     */
    public ElevatorSimProfile(final String simName,
                            final TalonFX talon,
                            final CANcoder cancoder,
                            final MotorSimConfiguration motorConst,
                            final ElevatorSimConfiguration elevConst) {
        this.mName = simName;
        this.mTalon = talon;
        this.mCANcoder = cancoder;
        this.mMotorConst = motorConst;
        this.mElevConst = elevConst;

        DCMotor elevatorGearbox = mMotorConst.simMotorModelSupplier.get();

        // Create sim object
        this.mElevatorSim = new ElevatorSim(
          elevatorGearbox,
          mElevConst.kElevatorGearing,
          mElevConst.kCarriageMass,
          mElevConst.kElevatorDrumRadius,
          mElevConst.kMinElevatorHeight,
          mElevConst.kMaxElevatorHeight,
          true,
          mElevConst.kDefaultSetpoint
        );

        // Create sim mechanism
        this.mElevatorMech = new MotionProfiledElevatorMechanism(mName);
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
        mElevatorSim.setInputVoltage(simState.getMotorVoltage());
        // Update the Elevator Sim each time throuhgh the loop
        mElevatorSim.update(getPeriod());

        // Get current position and velocity of the Elevator Sim ...
        double position_meters = mElevatorSim.getPositionMeters();
        double velocity_mps = mElevatorSim.getVelocityMetersPerSecond();

        // ... and set the position and velocity for the lead motor simulation 
        // (Multiply elevator positon by total gearing reduction from motor to elevator)
        simState.setRawRotorPosition(position_meters * mElevConst.kElevatorGearing);
        simState.setRotorVelocity(velocity_mps * mElevConst.kElevatorGearing);

        // If using an external encoder, update its sim as well
        if (mCANcoder != null) {
            // (Multiply elevator position by gearing reduction from sensor to elevator)
            mCANcoder.getSimState().setRawPosition(position_meters * mElevConst.kSensorReduction);
            mCANcoder.getSimState().setVelocity(velocity_mps * mElevConst.kSensorReduction);
        }

        // Update elevator sim mechanism
        mElevatorMech.update(position_meters);

    }
}
