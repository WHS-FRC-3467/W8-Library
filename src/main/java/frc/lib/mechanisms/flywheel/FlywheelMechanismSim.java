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

package frc.lib.mechanisms.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.util.Color;

import frc.lib.io.motor.MotorIOSim;
import frc.lib.mechanisms.MotorSimCalcs;
import frc.lib.util.BatteryModel;

/**
 * A simulated implementation of the FlywheelMechanism abstract class that uses FlywheelSim to
 * simulate the behavior of a flywheel mechanism.
 */
public class FlywheelMechanismSim extends FlywheelMechanism<MotorIOSim> {
    private final FlywheelSim sim;
    private final FlywheelVisualizer visualizer;
    private final AngularVelocity tolerance;
    private final double rotorToMechanismRatio;

    private Time lastTime = RobotController.getMeasureTime();
    private AngularVelocity lastVelocity = RadiansPerSecond.zero();
    private Angle simPosition = Radians.zero();

    /**
     * Constructs a simulated flywheel mechanism with the given parameters. 
     * 
     * @param name Name of the flywheel mechanism
     * @param io MotorIOSim implementation (e.g. TalonFX) for the flywheel
     * @param motor The bare (unreduced) DCMotor model representing the entire motor group, 
     * including all leaders and followers. For example, DCMotor.getKrakenX60Foc(2) for two motors.
     * Gear reduction is applied separately, internal to this simulation, using the provided ratio.
     * @param rotorToMechanismRatio The ratio of the rotor to mechanism movement. This value should match what is 
     * provided in MotorIO implementation's config file. An error will be thrown if the MotorIO implementation reports 
     * a different ratio than what is provided here.
     * @param momentOfInertia The moment of inertia of the flywheel in kg*m^2
     * @param tolerance The acceptable angular velocity error tolerance for the flywheel in rad/s
     */
    public FlywheelMechanismSim(
            String name,
            MotorIOSim io,
            DCMotor motor,
            double rotorToMechanismRatio,
            MomentOfInertia momentOfInertia,
            AngularVelocity tolerance) {
        super(name, io);
        this.rotorToMechanismRatio = rotorToMechanismRatio; 

        if (momentOfInertia.isEquivalent(KilogramSquareMeters.zero()))
            throw new IllegalArgumentException("momentOfInertia must be greater than zero!");

        this.tolerance = tolerance;
        /* 
         * Both the plant and sim API require an unreduced DCMotor object with the proper number of motors to properly 
         * model the gearbox; gearing is manually applied. 
         */
        sim =
                new FlywheelSim(
                        LinearSystemId.createFlywheelSystem(
                                motor,
                                momentOfInertia.in(KilogramSquareMeters),
                                rotorToMechanismRatio),
                        motor);

        visualizer = new FlywheelVisualizer(name);
    }

    @Override
    public void periodic() {
        Time currentTime = RobotController.getMeasureTime();
        double deltaTime = currentTime.minus(lastTime).in(Seconds);

        sim.setInputVoltage(inputs.appliedVoltage.in(Volts));
        sim.update(deltaTime);
        /* 
         * Calculate the motor group's supply current. PowerProfiler independently reports the supply current provided by CTRE 
         * simulation. Therefore, this value and PowerProfiler's may slightly disagree in sim, for which this value is currently 
         * treated as authoritative. 
         * 
         * Note this value cannot be trusted in the first robot loop while MotorInputs is initializing (e.g. numMotors defaults to 1).
         */
        double supplyCurrentAmps = MotorSimCalcs.getSupplyCurrentAmps(RobotController.getBatteryVoltage(), inputs.appliedVoltage.in(Volts), 
        inputs.torqueCurrent.in(Amps)) * getNumberOfMotors();
        BatteryModel.addCurrentLoad(supplyCurrentAmps);

        // Angular displacement kinematic equation (trapezoidal integration of theta)
        AngularVelocity currentVelocity = sim.getAngularVelocity();
        Angle positionChange =
                (lastVelocity.plus(currentVelocity).times(Seconds.of(deltaTime))).times(0.5);

        lastTime = currentTime;
        simPosition = simPosition.plus(positionChange);
        lastVelocity = currentVelocity;

        io.setMechanismPosition(simPosition);
        io.setMechanismVelocity(currentVelocity);
        io.setMechanismAcceleration(sim.getAngularAcceleration());

        /* 
         * Enforce consistency between the gear ratio used to construct this mechanism's state-space model and that reported by its 
         * corresponding MotorIO implementation; after the first mechanism.periodic() call below, the corresponding MotorIO implementation's 
         * updateInputs() method will also be called, thereby correctly setting the motor parameters (e.g. rotorToMechanismRatio) in the 
         * MotorIO.MotorInputs structure. Before then, these parameters will have their default values from the MotorIO interface (e.g. 
         * rotorToMechanismRatio = 1.0). 
         * 
         * Note that including the motor parameters as inputs rather than IO getters is for consistency with replay.
         */
        super.periodic();

        if (!Double.isFinite(rotorToMechanismRatio)
                || rotorToMechanismRatio <= 0.0
                || !Double.isFinite(getRotorToMechanismRatio())
                || getRotorToMechanismRatio() <= 0.0
                || Math.abs(rotorToMechanismRatio - getRotorToMechanismRatio()) > 1e-6) {
            throw new IllegalStateException(
                    String.format(
                            "Inconsistent/invalid rotor-to-mechanism ratio: "
                                    + "MotorIO reports %.6f, simulation configured %.6f",
                            getRotorToMechanismRatio(),
                            rotorToMechanismRatio));
        }

        visualizer.setAngle(inputs.position);
        if (inputs.velocityError != null && inputs.velocityError.lte(tolerance)) {
            visualizer.setColor(Color.kGreen);
        } else {
            visualizer.setColor(Color.kBlack);
        }
    }

    @Override
    public void setEncoderPosition(Angle position) {
        simPosition = position;
    }
}
