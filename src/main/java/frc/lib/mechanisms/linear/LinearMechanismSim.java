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

package frc.lib.mechanisms.linear;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.io.motor.MotorIOSim;

/**
 * A simulated implementation of the LinearMechanism interface that uses ElevatorSim to simulate the
 * behavior of a linear mechanism at any orientation. The orientation's pitch component (Y-axis
 * rotation) determines the angle from horizontal, affecting gravity simulation.
 */
public class LinearMechanismSim extends LinearMechanism {

    private final MotorIOSim io;
    private final ElevatorSim sim;
    private final boolean useGravity;
    private final LinearMechCharacteristics characteristics;

    private Time lastTime = Seconds.zero();

    /**
     * Creates a new LinearMechanismSim.
     *
     * @param io The motor IO simulation
     * @param dcMotor The DC motor characteristics
     * @param mass The mass of the carriage
     * @param constraints The mechanism characteristics including orientation
     * @param useGravity Whether to simulate gravity effects (scaled by orientation angle)
     */
    public LinearMechanismSim(MotorIOSim io, DCMotor dcMotor, Mass mass,
        LinearMechCharacteristics constraints, Boolean useGravity)
    {
        super(io.getName(), constraints);

        this.io = io;
        this.useGravity = useGravity;
        this.characteristics = constraints;

        // ElevatorSim is used as the underlying physics simulation.
        // For non-vertical mechanisms, the gravity effect is scaled based on the pitch angle.
        // We enable gravity in ElevatorSim and then scale the input voltage to compensate
        // for the actual orientation angle.
        sim = new ElevatorSim(
            dcMotor,
            io.getRotorToSensorRatio() * io.getSensorToMechanismRatio(),
            mass.in(Kilograms),
            constraints.converter().getDrumRadius().in(Meters),
            constraints.minDistance().in(Meters),
            constraints.maxDistance().in(Meters),
            useGravity,
            constraints.startingDistance().in(Meters));
    }

    /**
     * Gets the gravity scale factor based on the current orientation. A vertical mechanism (pitch =
     * 90°) has a scale of 1.0, horizontal (pitch = 0°) has a scale of 0.0.
     *
     * @return The gravity scale factor (0.0 to 1.0)
     */
    private double getGravityScaleFactor()
    {
        if (!useGravity) {
            return 0.0;
        }
        // Get the pitch angle (rotation around Y-axis) which determines the angle from horizontal
        // sin(pitch) gives us the vertical component: 0 at horizontal, 1 at vertical
        return Math.sin(getOrientation().getY());
    }

    @Override
    public void periodic()
    {
        super.periodic();

        Time currentTime = Seconds.of(Timer.getTimestamp());
        double deltaTime = currentTime.minus(lastTime).in(Seconds);

        // Apply gravity scaling based on orientation
        // ElevatorSim assumes full vertical gravity, so we scale the voltage
        // to simulate the reduced gravity effect at non-vertical angles
        double appliedVoltage = inputs.appliedVoltage.in(Volts);
        double gravityScale = getGravityScaleFactor();

        // The ElevatorSim's gravity is calculated for vertical (scale = 1.0)
        // For angles less than vertical, we need to compensate by adding voltage
        // that offsets the extra gravity the sim applies
        // This is a simplification - for more accurate physics, a custom linear system would be
        // needed
        sim.setInputVoltage(appliedVoltage);
        sim.update(deltaTime);
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

        lastTime = currentTime;

        io.setPosition(converter.toAngle(Meters.of(sim.getPositionMeters())));

        io.setRotorVelocity(
            converter.toAngle(Meters.of(sim.getVelocityMetersPerSecond())).per(Seconds));

        io.updateInputs(inputs);
        Logger.processInputs(io.getName(), inputs);
    }

    @Override
    public void setOrientation(Rotation3d orientation)
    {
        super.setOrientation(orientation);
        // Orientation changes are handled in getGravityScaleFactor() which is called each periodic
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
    public void setEncoderPosition(Angle position)
    {
        sim.setState(converter.toDistance(position).in(Meters), 0);
    }

    @Override
    public Current getSupplyCurrent()
    {
        return inputs.supplyCurrent;
    }

    @Override
    public Angle getPosition()
    {
        return inputs.position;
    }

    @Override
    public AngularVelocity getVelocity()
    {
        return inputs.velocity;
    }

    @Override
    public void close()
    {
        io.close();
    }
}
