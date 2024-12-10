package frc.robot.util.sim;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.sim.PhysicsSim.SimProfile;

/**
 * Holds information about a simulated TalonFX.
 */
class TalonFXSimProfile extends SimProfile {

    private final TalonFX mTalon;
    private final DCMotorSim mMotorSim;

    /**
     * Creates a new simulation profile for a TalonFX device
     * using a KrakenX60Foc motor sim.
     * 
     * @param talon
     *      The TalonFX device
     * @param reduction
     *      The gearing of the DC motor (numbers greater than 1 represent reductions).
     * @param rotorInertia
     *      Rotational Inertia of the mechanism at the rotor
     */
    public TalonFXSimProfile(final TalonFX talon, final MotorSimConfiguration constants) {
        this.mTalon = talon;
        this.mMotorSim = new DCMotorSim(constants.simMotorModelSupplier.get(),
				                        constants.simReduction,
                                        constants.simMOI
        );
    }

    /**
     * Creates a new simulation profile for a TalonFX device
     *  with provided DCMotorSim.
     * 
     * @param talon
     *      The TalonFX device
     * @param motorModel
     *      A DCMotorSim
     */
    public TalonFXSimProfile(final TalonFX talon, final DCMotorSim motorModel) {
        this.mTalon = talon;
        this.mMotorSim = motorModel;
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

        // Set the input (voltage) to the Motor Simulation
        mMotorSim.setInputVoltage(simState.getMotorVoltage());
        // Update the Motor Sim each time through the loop
        mMotorSim.update(getPeriod());

        // Get current position and velocity of the Motor Sim ...
        final double position_rot = mMotorSim.getAngularPositionRotations();
        final double velocity_rps = Units.radiansToRotations(mMotorSim.getAngularVelocityRadPerSec());

        // ... and set the position and velocity for the lead motor simulation 
        simState.setRawRotorPosition(position_rot);
        simState.setRotorVelocity(velocity_rps);
        
    }
}