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
     * Gear reduction is applied separately using the MotorIO ratios.
     * @param momentOfInertia The moment of inertia of the flywheel in kg*m^2
     * @param tolerance The acceptable angular velocity error tolerance for the flywheel in rad/s
     */
    public FlywheelMechanismSim(
            String name,
            MotorIOSim io,
            DCMotor motor,
            MomentOfInertia momentOfInertia,
            AngularVelocity tolerance) {
        super(name, io);

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
                                io.getRotorToMechanismRatio()),
                        motor);

        visualizer = new FlywheelVisualizer(name);
    }

    @Override
    public void periodic() {
        Time currentTime = RobotController.getMeasureTime();
        double deltaTime = currentTime.minus(lastTime).in(Seconds);

        sim.setInputVoltage(inputs.appliedVoltage.in(Volts));
        sim.update(deltaTime);
        // Calculate the motor group's supply current 
        double supplyCurrentAmps = MotorSimCalcs.getSupplyCurrentAmps(RobotController.getBatteryVoltage(), inputs.appliedVoltage.in(Volts), 
        inputs.torqueCurrent.in(Amps)) * io.getNumberOfMotors();
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

        super.periodic();

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
