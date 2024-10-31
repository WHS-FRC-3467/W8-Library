package frc.robot.util.sim;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.sim.PhysicsSim.SimProfile;

/**
 * Holds information about a simulated TalonFX.
 */
class TalonFXSimProfile extends SimProfile {
    private static final double kMotorResistance = 0.002; // Assume 2mOhm resistance for voltage drop calculation
    private final TalonFX mTalon;

    private final DCMotorSim _motorSim;

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
    public TalonFXSimProfile(final TalonFX talon, final double reduction, final double rotorInertia) {
        this.mTalon = talon;
        this._motorSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), reduction, rotorInertia);
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
        this._motorSim = motorModel;
    }

    /**
     * Runs the simulation profile.
     * 
     * This uses very rudimentary physics simulation and exists to allow users to
     * test features of our products in simulation using our examples out of the
     * box. Users may modify this to utilize more accurate physics simulation.
     */
    public void run() {
        /// DEVICE SPEED SIMULATION

        _motorSim.setInputVoltage(mTalon.getSimState().getMotorVoltage());

        _motorSim.update(getPeriod());

        /// SET SIM PHYSICS INPUTS
        final double position_rot = _motorSim.getAngularPositionRotations();
        final double velocity_rps = Units.radiansToRotations(_motorSim.getAngularVelocityRadPerSec());

        mTalon.getSimState().setRawRotorPosition(position_rot);
        mTalon.getSimState().setRotorVelocity(velocity_rps);

        mTalon.getSimState().setSupplyVoltage(12 - mTalon.getSimState().getSupplyCurrent() * kMotorResistance);
    }
}