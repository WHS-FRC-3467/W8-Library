// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.mechanisms.rotary;


import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.system.plant.DCMotor;
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
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.io.motor.MotorIOSim;
import frc.lib.io.motor.MotorInputsAutoLogged;

/** Add your docs here. */
public class RotaryMechanismSim implements RotaryMechanism {

    private final MotorIOSim io;
    private final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();
    private final SingleJointedArmSim sim;

    RotaryVisualizer visualizer;

    private Time lastTime = Seconds.zero();

    public RotaryMechanismSim(MotorIOSim io, DCMotor dcMotor,
        MomentOfInertia momentOfInertia, Boolean useGravity,
        RotaryMechCharacteristics characteristics)
    {
        if (momentOfInertia.isEquivalent(KilogramSquareMeters.zero()))
            throw new IllegalArgumentException(
                "momentOfInertia must be greater than zero!");

        this.io = io;
        sim = new SingleJointedArmSim(
            dcMotor,
            io.getGearRatio(),
            momentOfInertia.in(KilogramSquareMeters),
            characteristics.armLength().in(Meters),
            characteristics.minAngle().in(Radians),
            characteristics.maxAngle().in(Radians),
            useGravity,
            characteristics.startingAngle().in(Radians));

        visualizer = new RotaryVisualizer(io.getName(), characteristics);
    }

    @Override
    public void periodic()
    {
        Time currentTime = Seconds.of(Timer.getTimestamp());
        double deltaTime = currentTime.minus(lastTime).in(Seconds);

        sim.setInputVoltage(inputs.appliedVoltage.in(Volts));
        sim.update(deltaTime);

        lastTime = currentTime;

        io.setPosition(Radians.of(sim.getAngleRads()));
        io.setRotorVelocity(
            RadiansPerSecond.of(sim.getVelocityRadPerSec()));

        io.updateInputs(inputs);
        Logger.processInputs(io.getName(), inputs);

        visualizer.setCurrentAngle(Radians.of(sim.getAngleRads()));
        if (inputs.activeTrajectoryPosition != null) {
            visualizer.setTrajectoryAngle(inputs.activeTrajectoryPosition);
        } else {
            visualizer.setTrajectoryAngle(Radians.of(sim.getAngleRads()));
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
        visualizer.setGoalAngle(position);
    }

    @Override
    public void runVelocity(AngularVelocity velocity, AngularAcceleration acceleration,
        PIDSlot slot)
    {
        io.runVelocity(velocity, acceleration, slot);
    }
}
