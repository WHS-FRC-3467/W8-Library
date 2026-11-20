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

package frc.lib.actuators.flywheel;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import frc.lib.actuators.MotorSimCalcs;
import frc.lib.io.motor.MotorIOSim;
import frc.lib.util.BatteryModel;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.MomentOfInertia;
import org.wpilib.units.measure.Time;
import org.wpilib.util.Color;

/**
 * A simulated implementation of the FlywheelMechanism abstract class that uses FlywheelSim to
 * simulate the behavior of a flywheel mechanism.
 */
public class FlywheelActuatorSim extends FlywheelActuator<MotorIOSim> {
    private final FlywheelSim sim;
    private final FlywheelActuatorVisualizer visualizer;
    private final AngularVelocity tolerance;
    private final double rotorToMechanismRatio;
    private final int numMotors;

    private Time lastTime = RobotController.getMeasureTime();
    private AngularVelocity lastVelocity = RadiansPerSecond.zero();
    private Angle simPosition = Radians.zero();

    /**
     * Constructs a simulated flywheel mechanism with the given parameters.
     *
     * @param name Name of the flywheel mechanism
     * @param io MotorIOSim implementation (e.g. TalonFX) for the flywheel
     * @param motor The bare (unreduced) DCMotor model representing the entire motor group,
     *     including the leaders + all followers. For example, DCMotor.getKrakenX60Foc(numMotors)
     *     for numMotors -- this is the same numMotors provided as a separate parameter in this
     *     call. Gear reduction is applied separately, internal to this simulation, using the
     *     provided ratio.
     * @param rotorToMechanismRatio The ratio of the rotor to mechanism movement. This value should
     *     match what is provided in MotorIO implementation's config file. An error will be thrown
     *     if the MotorIO implementation reports a different ratio than what is provided here.
     * @param numMotors The number of motors (leader + all followers) utilized in this mechanism.
     *     This is the same value provided separately to the motor parameter. Additionally, this
     *     parameter should match what is provided in MotorIO implementation's config file. An error
     *     will be thrown if the MotorIO implementation reports a different number than what is
     *     provided here.
     * @param momentOfInertia The moment of inertia of the flywheel in kg*m^2
     * @param tolerance The acceptable angular velocity error tolerance for the flywheel in rad/s
     */
    public FlywheelActuatorSim(
            String name,
            MotorIOSim io,
            DCMotor motor,
            double rotorToMechanismRatio,
            int numMotors,
            MomentOfInertia momentOfInertia,
            AngularVelocity tolerance) {
        super(name, io);

        if (!Double.isFinite(rotorToMechanismRatio) || rotorToMechanismRatio <= 0.0) {
            throw new IllegalArgumentException(
                    "Rotor-to-mechanism ratio must be greater than zero and finite.");
        }

        if (numMotors < 1) {
            throw new IllegalArgumentException(
                    "Number of motors in the motor group must be greater than or equal to one.");
        }

        if (momentOfInertia.lte(KilogramSquareMeters.zero()))
            throw new IllegalArgumentException("momentOfInertia must be greater than zero!");

        this.rotorToMechanismRatio = rotorToMechanismRatio;
        this.numMotors = numMotors;
        this.tolerance = tolerance;

        /*
         * Both the sim API and plant require an unreduced DCMotor object with the proper number of motors to properly
         * model the gearbox; gearing is manually applied.
         */
        sim =
                new FlywheelSim(
                        Models.flywheelFromPhysicalConstants(
                                motor,
                                momentOfInertia.in(KilogramSquareMeters),
                                rotorToMechanismRatio),
                        motor);

        visualizer = new FlywheelActuatorVisualizer(name);
    }

    @Override
    public void periodic() {
        Time currentTime = RobotController.getMeasureTime();
        double deltaTime = currentTime.minus(lastTime).in(Seconds);

        sim.setInputVoltage(inputs.appliedVoltage.in(Volts));
        sim.update(deltaTime);
        /*
         * Calculate the motor group's supply current for BatteryModel loading.
         * PowerProfiler independently reports CTRE's simulated supply current,
         * so the two estimates may differ. The MotorSimCalcs estimate is treated
         * as authoritative for battery simulation.
         */
        double supplyCurrentAmps =
                MotorSimCalcs.getSupplyCurrentAmps(
                                RobotController.getBatteryVoltage(),
                                inputs.appliedVoltage.in(Volts),
                                inputs.torqueCurrent.in(Amps))
                        * numMotors;
        BatteryModel.addCurrentLoad(supplyCurrentAmps);

        // Angular displacement kinematic equation (trapezoidal integration of theta)
        AngularVelocity currentVelocity = RadiansPerSecond.of(sim.getAngularVelocity());
        Angle positionChange =
                (lastVelocity.plus(currentVelocity).times(Seconds.of(deltaTime))).times(0.5);

        lastTime = currentTime;
        simPosition = simPosition.plus(positionChange);
        lastVelocity = currentVelocity;

        io.setMechanismPosition(simPosition);
        io.setMechanismVelocity(currentVelocity);
        io.setMechanismAcceleration(RadiansPerSecondPerSecond.of(sim.getAngularAcceleration()));

        /*
         * Enforce consistency between the gear ratio and number of motors used to construct this mechanism's state-space model and that
         * reported by its corresponding MotorIO implementation; after the first mechanism.periodic() call below, the corresponding MotorIO
         * implementation's updateInputs() method will also be called, thereby correctly setting the motor parameters (e.g. rotorToMechanismRatio)
         * in the MotorIO.MotorIOInputs structure. Before then, these parameters will have their default values from the MotorIO interface (e.g.
         * rotorToMechanismRatio = 1.0, numMotors = 1).
         *
         * Note that including the motor parameters as inputs rather than IO getters is for consistency with replay.
         */
        super.periodic();

        if (!Double.isFinite(getRotorToMechanismRatio())
                || getRotorToMechanismRatio() <= 0.0
                || Math.abs(rotorToMechanismRatio - getRotorToMechanismRatio()) > 1e-6) {
            throw new IllegalStateException(
                    String.format(
                            "Inconsistent/invalid rotor-to-mechanism ratio: "
                                    + "MotorIO reports %.6f, simulation configured %.6f",
                            getRotorToMechanismRatio(), rotorToMechanismRatio));
        }

        if (numMotors != getNumberOfMotors()) {
            throw new IllegalStateException(
                    String.format(
                            "Inconsistent motor count: "
                                    + "MotorIO reports %d, simulation configured %d",
                            getNumberOfMotors(), numMotors));
        }

        visualizer.setAngle(inputs.position);
        if (inputs.velocityError != null
                && inputs.velocityError.isNear(RadiansPerSecond.zero(), tolerance)) {
            visualizer.setColor(Color.GREEN);
        } else {
            visualizer.setColor(Color.BLACK);
        }
    }

    @Override
    public void setEncoderPosition(Angle position) {
        simPosition = position;
    }
}
