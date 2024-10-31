package frc.robot.util.sim;

import java.util.ArrayList;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Manages physics simulation for CTRE products.
 */
public class PhysicsSim {
    private static final PhysicsSim sim = new PhysicsSim();

    /**
     * Gets the robot simulator instance.
     */
    public static PhysicsSim getInstance() {
        return sim;
    }

    /**
     * Adds a TalonFX controller to the simulator.
     * 
     * @param talon
     *        The TalonFX device
     * @param reduction
     *      The gearing of the DC motor (numbers greater than 1 represent reductions).
     * @param rotorInertia
     *        Rotational Inertia of the mechanism at the rotor
     */
    public void addTalonFX(TalonFX talon, final double reduction, final double rotorInertia) {
        if (talon != null) {
            TalonFXSimProfile simFalcon = new TalonFXSimProfile(talon, reduction, rotorInertia);
            _simProfiles.add(simFalcon);
        }
    }

    /**
     * Adds a TalonFX controller to the simulator
     *  using provided TalonFX object and DCMotorSim
     * 
     * @param talon
     *        The TalonFX device
     * @param motorModel
     *      A DCMotorSim
     */
    public void addTalonFX(TalonFX talon, final DCMotorSim motorModel) {
        if (talon != null) {
            TalonFXSimProfile simFalcon = new TalonFXSimProfile(talon, motorModel);
            _simProfiles.add(simFalcon);
        }
    }

    /**
     * Runs the simulator:
     * - enable the robot
     * - simulate sensors
     */
    public void run() {
        // Simulate devices
        for (SimProfile simProfile : _simProfiles) {
            simProfile.run();
        }
    }

    private final ArrayList<SimProfile> _simProfiles = new ArrayList<SimProfile>();

    /**
     * Holds information about a simulated device.
     */
    static class SimProfile {
        private double _lastTime;
        private boolean _running = false;

        /**
         * Runs the simulation profile.
         * Implemented by device-specific profiles.
         */
        public void run() {
        }

        /**
         * Returns the time since last call, in seconds.
         */
        protected double getPeriod() {
            // set the start time if not yet running
            if (!_running) {
                _lastTime = Utils.getCurrentTimeSeconds();
                _running = true;
            }

            double now = Utils.getCurrentTimeSeconds();
            final double period = now - _lastTime;
            _lastTime = now;

            return period;
        }
    }
}