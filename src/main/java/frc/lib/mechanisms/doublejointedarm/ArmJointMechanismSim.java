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
package frc.lib.mechanisms.doublejointedarm;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

import frc.lib.io.absoluteencoder.AbsoluteEncoderIOSim;
import frc.lib.io.motor.MotorIOSim;

import java.util.Optional;

public class ArmJointMechanismSim extends ArmJointMechanism<MotorIOSim, AbsoluteEncoderIOSim> {
    private final ArmJointSim sim;

    private Time lastTime = RobotController.getMeasureTime();

    public ArmJointMechanismSim(
            String name,
            MotorIOSim io,
            DCMotor motor,
            MomentOfInertia momentOfInertia,
            Boolean useGravity,
            ArmJointMechanism.JointCharacteristics characteristics,
            Optional<AbsoluteEncoderIOSim> absoluteEncoder,
            String encoderName) {
        super(name, characteristics, io, absoluteEncoder, encoderName);

        if (momentOfInertia.isEquivalent(KilogramSquareMeters.zero()))
            throw new IllegalArgumentException("momentOfInertia must be greater than zero!");

        sim =
                new ArmJointSim(
                        motor,
                        io.getRotorToSensorRatio() * io.getSensorToMechanismRatio(),
                        momentOfInertia.in(KilogramSquareMeters),
                        characteristics.armLength().in(Meters),
                        characteristics.minAngle().in(Radians),
                        characteristics.maxAngle().in(Radians),
                        useGravity,
                        characteristics.startingAngle().in(Radians));
    }

    public void updateSimTopVelocity(AngularVelocity vel) {
        sim.setTopVelocity(Optional.of(vel));
    }

    public void updateSimLowerAngle(double angle) {

        sim.setBottomAngle(Optional.of(angle));
    }

    @Override
    public void periodic() {
        Time currentTime = RobotController.getMeasureTime();
        double deltaTime = currentTime.minus(lastTime).in(Seconds);

        sim.setInputVoltage(inputs.appliedVoltage.in(Volts));

        sim.update(deltaTime);
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

        lastTime = currentTime;

        io.setPosition(Radians.of(sim.getAngleRads()));
        io.setRotorVelocity(
                RadiansPerSecond.of(sim.getVelocityRadPerSec())
                        .times(io.getRotorToSensorRatio() * io.getSensorToMechanismRatio()));

        absoluteEncoder.ifPresent(
                encoderSim -> {
                    encoderSim.setAngle(
                            Radians.of(sim.getAngleRads()).times(io.getSensorToMechanismRatio()));
                });

        super.periodic();
    }
}
