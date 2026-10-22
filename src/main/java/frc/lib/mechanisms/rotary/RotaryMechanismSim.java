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
import frc.lib.mechanisms.MotorSimCalcs;
import frc.lib.util.BatteryModel;

import java.util.Optional;

/**
 * A simulated implementation of the RotaryMechanism base class that uses SingleJointedArmSim to
 * simulate the behavior of a rotary mechanism.
 */
public class RotaryMechanismSim extends RotaryMechanism<MotorIOSim, AbsoluteEncoderIOSim> {
    private final SingleJointedArmSim sim;

    private Time lastTime = RobotController.getMeasureTime();
    private AngularVelocity lastVelocity = RadiansPerSecond.zero();
    private final double rotorToMechanismRatio;

    /**
     * Constructs a simulated rotary mechanism with the given parameters. 
     * 
     * @param name Name of the rotary mechanism
     * @param io MotorIOSim implementation (e.g. TalonFX) for the rotor
     * @param motor The bare (unreduced) DCMotor model representing the entire motor group, 
     * including all leaders and followers. For example, DCMotor.getKrakenX60Foc(2) for two motors.
     * Gear reduction is applied separately, internal to this simulation, using the provided ratio.
     * @param rotorToMechanismRatio The ratio of the rotor to mechanism movement. This value should match what is 
     * provided in MotorIO implementation's config file. An error will be thrown if the MotorIO implementation reports 
     * a different ratio than what is provided here.
     * @param momentOfInertia The moment of inertia of the rotor in kg*m^2
     * @param useGravity Whether to simulate gravity effects (applies when orientation is vertical)
     * @param characteristics RotaryMechCharacteristics object containing arm length, min/max angles, starting 
     * angle, and axis
     * @param absoluteEncoder Optional AbsoluteEncoderIOSim implementation for absolute encoder feedback
     * @param encoderName Name of the absolute encoder (used for logging)
     */
    public RotaryMechanismSim(
            String name,
            MotorIOSim io,
            DCMotor motor,
            double rotorToMechanismRatio,
            MomentOfInertia momentOfInertia,
            Boolean useGravity,
            RotaryMechCharacteristics characteristics,
            Optional<AbsoluteEncoderIOSim> absoluteEncoder,
            String encoderName) {
        super(name, characteristics, io, absoluteEncoder, encoderName);
        this.rotorToMechanismRatio = rotorToMechanismRatio;

        if (momentOfInertia.isEquivalent(KilogramSquareMeters.zero()))
            throw new IllegalArgumentException("momentOfInertia must be greater than zero!");

        sim =
                new SingleJointedArmSim(
                        motor,
                        rotorToMechanismRatio,
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
        // Calculate the motor group's supply current
        // Note this value cannot be trusted in the first robot loop while MotorInputs is initializing (e.g. numMotors defaults to 1)
        double supplyCurrentAmps = MotorSimCalcs.getSupplyCurrentAmps(RobotController.getBatteryVoltage(), inputs.appliedVoltage.in(Volts), 
        inputs.torqueCurrent.in(Amps)) * getNumberOfMotors();
        BatteryModel.addCurrentLoad(supplyCurrentAmps);

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
    }

    @Override
    public void setEncoderPosition(Angle position) {
        sim.setState(position.in(Radians), sim.getVelocityRadPerSec());
    }
}
