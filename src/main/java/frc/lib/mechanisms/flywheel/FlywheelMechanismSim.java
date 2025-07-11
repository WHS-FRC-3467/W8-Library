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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.io.motor.MotorIOSim;

public class FlywheelMechanismSim extends FlywheelMechanism<MotorIOSim> {
    private final FlywheelSim sim;

    private Time lastTime;

    public FlywheelMechanismSim(MotorIOSim io, DCMotor characteristics,
        MomentOfInertia momentOfInertia)
    {
        super(io);
        sim = new FlywheelSim(LinearSystemId.createFlywheelSystem(characteristics,
            momentOfInertia.in(KilogramSquareMeters),
            io.getGearRatio()), characteristics);
    }

    @Override
    public void periodic()
    {
        Time currentTime = Seconds.of(Timer.getTimestamp());
        double deltaTime = currentTime.minus(lastTime).in(Seconds);

        sim.setInputVoltage(super.inputs.appliedVoltage.in(Volts));
        sim.update(deltaTime);

        lastTime = currentTime;

        super.io.setRotorVelocity(sim.getAngularVelocity());
        super.io.setRotorAcceleration(sim.getAngularAcceleration());

        // Angular displacement kinematic equation (θ = ω₀t + (1/2)αt²)'
        Angle positionChange = Radians.of(sim.getAngularVelocityRadPerSec() * deltaTime
            + 0.5 * sim.getAngularAccelerationRadPerSecSq() * Math.pow(deltaTime, 2));

        super.io.setPosition(super.inputs.position.plus(positionChange));

        super.periodic();
    }
}
