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

import frc.lib.io.motor.MotorIOSim;
import frc.lib.util.BatterySimCurrentAccumulator;

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

    private Time lastTime = RobotController.getMeasureTime();
    private AngularVelocity lastVelocity = RadiansPerSecond.zero();
    private Angle simPosition = Radians.zero();

    public FlywheelActuatorSim(
            String name,
            MotorIOSim io,
            DCMotor motor,
            MomentOfInertia momentOfInertia,
            AngularVelocity tolerance) {
        super(name, io);

        if (momentOfInertia.isEquivalent(KilogramSquareMeters.zero()))
            throw new IllegalArgumentException("momentOfInertia must be greater than zero!");

        this.tolerance = tolerance;
        sim =
                new FlywheelSim(
                        Models.flywheelFromPhysicalConstants(
                                motor,
                                momentOfInertia.in(KilogramSquareMeters),
                                io.getRotorToSensorRatio() * io.getSensorToMechanismRatio()),
                        motor);

        visualizer = new FlywheelActuatorVisualizer(name);
    }

    @Override
    public void periodic() {
        Time currentTime = RobotController.getMeasureTime();
        double deltaTime = currentTime.minus(lastTime).in(Seconds);

        sim.setInputVoltage(inputs.appliedVoltage.in(Volts));
        sim.update(deltaTime);
        BatterySimCurrentAccumulator.addCurrentLoad(Amps.of(sim.getCurrentDraw()));

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

        super.periodic();

        visualizer.setAngle(inputs.position);
        if (inputs.velocityError != null && inputs.velocityError.lte(tolerance)) {
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
