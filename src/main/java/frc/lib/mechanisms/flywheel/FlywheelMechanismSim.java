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

package frc.lib.mechanisms.flywheel;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.io.motor.MotorIOSim;
import frc.lib.io.motor.MotorInputsAutoLogged;

/**
 * A simulated implementation of the FlywheelMechanism interface that uses FlywheelSim to simulate
 * the behavior of a flywheel mechanism.
 */
public class FlywheelMechanismSim implements FlywheelMechanism {

    private final MotorIOSim io;
    private final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();
    private final FlywheelSim sim;
    private final FlywheelVisualizer visualizer;
    private final AngularVelocity tolerance;

    private Time lastTime = Seconds.zero();

    public FlywheelMechanismSim(MotorIOSim io, DCMotor characteristics,
        MomentOfInertia momentOfInertia, AngularVelocity tolerance)
    {
        if (momentOfInertia.isEquivalent(KilogramSquareMeters.zero()))
            throw new IllegalArgumentException(
                "momentOfInertia must be greater than zero!");

        this.io = io;
        this.tolerance = tolerance;
        sim = new FlywheelSim(LinearSystemId.createFlywheelSystem(characteristics,
            momentOfInertia.in(KilogramSquareMeters),
            io.getGearRatio()), characteristics);

        visualizer = new FlywheelVisualizer(io.getName());
    }

    @Override
    public void periodic()
    {
        Time currentTime = Seconds.of(Timer.getTimestamp());
        double deltaTime = currentTime.minus(lastTime).in(Seconds);

        sim.setInputVoltage(inputs.appliedVoltage.in(Volts));
        sim.update(deltaTime);
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

        lastTime = currentTime;

        io.setRotorVelocity(sim.getAngularVelocity());
        io.setRotorAcceleration(sim.getAngularAcceleration());

        // Angular displacement kinematic equation (θ = ω₀t + (1/2)αt²)'
        Angle positionChange = Radians.of(sim.getAngularVelocityRadPerSec() * deltaTime
            + 0.5 * sim.getAngularAccelerationRadPerSecSq() * Math.pow(deltaTime, 2));

        io.setPosition(inputs.position.plus(positionChange));

        io.updateInputs(inputs);
        Logger.processInputs(io.getName(), inputs);

        visualizer.setAngle(inputs.position);
        if (inputs.velocityError != null) {
            if (inputs.velocityError.lte(tolerance)) {
                visualizer.setColor(Color.kGreen);
            }
        } else {
            visualizer.setColor(Color.kBlack);
        }
    }

    @Override
    public void runCoast()
    {
        io.runCoast();
    }

    @Override
    public void runBrake()
    {
        io.runBrake();
    }

    @Override
    public void runVoltage(Voltage voltage)
    {
        io.runVoltage(voltage);
    }

    @Override
    public void runCurrent(Current current)
    {
        io.runCurrent(current);
    }

    @Override
    public void runDutyCycle(double dutyCycle)
    {
        io.runDutyCycle(dutyCycle);
    }

    @Override
    public void runPosition(Angle position, AngularVelocity cruiseVelocity,
        AngularAcceleration acceleration,
        Velocity<AngularAccelerationUnit> maxJerk, PIDSlot slot)
    {
        io.runPosition(position, cruiseVelocity, acceleration, maxJerk, slot);
    }

    @Override
    public void runVelocity(AngularVelocity velocity, AngularAcceleration acceleration,
        PIDSlot slot)
    {
        io.runVelocity(velocity, acceleration, slot);
    }

    @Override
    public Current getTorqueCurrent()
    {
        return inputs.torqueCurrent;
    }

    @Override
    public AngularVelocity getVelocity() {
        return inputs.velocity;
    }

    @Override
    public void close()
    {
        io.close();
    }
}
