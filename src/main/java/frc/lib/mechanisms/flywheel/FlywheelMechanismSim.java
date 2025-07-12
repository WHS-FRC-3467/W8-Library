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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanismSim.PhysicsError.Cause;
import lombok.AllArgsConstructor;
import lombok.Getter;
import frc.lib.io.motor.MotorIOSim;
import frc.lib.io.motor.MotorInputsAutoLogged;

public class FlywheelMechanismSim implements FlywheelMechanism {
    public static class PhysicsError extends Exception {
        @Getter
        @AllArgsConstructor
        public enum Cause {
            LTE_ZERO("Cannot be less than or equal to zero");

            public final String message;
        }

        public PhysicsError(Cause cause)
        {
            super(cause.getMessage());
        }
    }

    private final MotorIOSim io;
    private final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();
    private final FlywheelSim sim;

    private Time lastTime = Seconds.zero();

    public FlywheelMechanismSim(MotorIOSim io, DCMotor characteristics,
        MomentOfInertia momentOfInertia) throws PhysicsError
    {
        if (momentOfInertia.isEquivalent(KilogramSquareMeters.zero()))
            throw new PhysicsError(Cause.LTE_ZERO);

        this.io = io;
        sim = new FlywheelSim(LinearSystemId.createFlywheelSystem(characteristics,
            momentOfInertia.in(KilogramSquareMeters),
            io.getGearRatio()), characteristics);
    }

    @Override
    public void periodic()
    {
        Time currentTime = Seconds.of(Timer.getTimestamp());
        double deltaTime = currentTime.minus(lastTime).in(Seconds);

        sim.setInputVoltage(inputs.appliedVoltage.in(Volts));
        sim.update(deltaTime);

        lastTime = currentTime;

        io.setRotorVelocity(sim.getAngularVelocity());
        io.setRotorAcceleration(sim.getAngularAcceleration());

        // Angular displacement kinematic equation (θ = ω₀t + (1/2)αt²)'
        Angle positionChange = Radians.of(sim.getAngularVelocityRadPerSec() * deltaTime
            + 0.5 * sim.getAngularAccelerationRadPerSecSq() * Math.pow(deltaTime, 2));

        io.setPosition(inputs.position.plus(positionChange));

        io.updateInputs(inputs);
        Logger.processInputs(io.getName(), inputs);
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
}
