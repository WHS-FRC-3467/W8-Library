// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.mechanisms.rotary;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import frc.lib.io.absoluteencoder.AbsoluteEncoderIOSim;
import frc.lib.io.motor.MotorIOSim;
import frc.lib.util.BatterySimCurrentAccumulator;

import java.util.Optional;

/**
 * A simulated implementation of the RotaryMechanism base class that uses SingleJointedArmSim to
 * simulate the behavior of a rotary mechanism.
 */
public class RotaryMechanismSim extends RotaryMechanism<MotorIOSim, AbsoluteEncoderIOSim> {
    private final SingleJointedArmSim sim;

    private Time lastTime = RobotController.getMeasureTime();
    private AngularVelocity lastVelocity = RadiansPerSecond.zero();

    public RotaryMechanismSim(
            String name,
            MotorIOSim io,
            DCMotor motor,
            MomentOfInertia momentOfInertia,
            Boolean useGravity,
            RotaryMechCharacteristics characteristics,
            Optional<AbsoluteEncoderIOSim> absoluteEncoder,
            String encoderName) {
        super(name, characteristics, io, absoluteEncoder, encoderName);

        if (momentOfInertia.isEquivalent(KilogramSquareMeters.zero()))
            throw new IllegalArgumentException("momentOfInertia must be greater than zero!");

        sim =
                new SingleJointedArmSim(
                        motor,
                        io.getRotorToSensorRatio() * io.getSensorToMechanismRatio(),
                        momentOfInertia.in(KilogramSquareMeters),
                        characteristics.armLength().in(Meters),
                        characteristics.minAngle().in(Radians),
                        characteristics.maxAngle().in(Radians),
                        useGravity,
                        characteristics.startingAngle().in(Radians));
    }

    @Override
    public void periodic() {
        Time currentTime = RobotController.getMeasureTime();
        double deltaTime = currentTime.minus(lastTime).in(Seconds);

        sim.setInputVoltage(inputs.appliedVoltage.in(Volts));
        sim.update(deltaTime);
        BatterySimCurrentAccumulator.addCurrentLoad(Amps.of(sim.getCurrentDrawAmps()));

        AngularVelocity currentVelocity = RadiansPerSecond.of(sim.getVelocityRadPerSec());
        AngularAcceleration currentAcceleration;

        if (deltaTime > 0) {
            currentAcceleration = currentVelocity.minus(lastVelocity).div(Seconds.of(deltaTime));
        } else {
            currentAcceleration = RadiansPerSecondPerSecond.zero();
        }

        lastTime = currentTime;
        lastVelocity = currentVelocity;

        io.setMechanismPosition(Radians.of(sim.getAngleRads()));
        io.setMechanismVelocity(currentVelocity);
        io.setMechanismAcceleration(currentAcceleration);

        absoluteEncoder.ifPresent(
                encoderSim -> {
                    encoderSim.setMechanismAngle(Radians.of(sim.getAngleRads()));
                });

        super.periodic();
    }

    @Override
    public void setEncoderPosition(Angle position) {
        sim.setState(position.in(Radians), sim.getVelocityRadPerSec());
    }
}
