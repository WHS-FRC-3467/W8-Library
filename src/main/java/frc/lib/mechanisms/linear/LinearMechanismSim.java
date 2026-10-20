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

package frc.lib.mechanisms.linear;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import frc.lib.io.motor.MotorIOSim;
import frc.lib.mechanisms.MotorSimCalcs;
import frc.lib.util.BatteryModel;

/**
 * A simulated implementation of the LinearMechanism interface that uses ElevatorSim to simulate the
 * behavior of a linear mechanism at any orientation. The orientation's pitch component (Y-axis
 * rotation) determines the angle from horizontal, affecting gravity simulation.
 */
public class LinearMechanismSim extends LinearMechanism<MotorIOSim> {
    private final ElevatorSim sim;

    private Time lastTime = RobotController.getMeasureTime();
    private AngularVelocity lastVelocity = RadiansPerSecond.zero();

    /**
     * Creates a new LinearMechanismSim.
     *
     * @param name The name of the mechanism
     * @param io The motor IO simulation
     * @param motor The bare (unreduced) DCMotor model representing the entire motor group, 
     * including all leaders and followers. For example, DCMotor.getKrakenX60Foc(2) for two motors.
     * Gear reduction is applied separately using the MotorIO ratios.
     * @param mass The mass of the carriage
     * @param useGravity Whether to simulate gravity effects (applies when orientation is vertical)
     * @param characteristics The mechanism characteristics including orientation
     */
    public LinearMechanismSim(
            String name,
            MotorIOSim io,
            DCMotor motor,
            Mass mass,
            Boolean useGravity,
            LinearMechCharacteristics characteristics) {
        super(name, characteristics, io);

        /* 
         * Both the underlying plant and sim API require an unreduced DCMotor object with the proper number of 
         * motors to properly model the gearbox; gearing is manually applied. 
         * 
         * Note: ElevatorSim assumes vertical orientation for gravity simulation.
         * The visualization and 3D pose will correctly reflect any orientation angle,
         * but physics simulation is most accurate when useGravity=true and orientation
         * is vertical (pitch = -90° for upward, 90° for downward), or when useGravity=false for
         * horizontal mechanisms.
         */
        sim =
                new ElevatorSim(
                        motor,
                        io.getRotorToMechanismRatio(),
                        mass.in(Kilograms),
                        characteristics.drumRadius().in(Meters),
                        characteristics.minDistance().in(Meters),
                        characteristics.maxDistance().in(Meters),
                        useGravity,
                        characteristics.startingDistance().in(Meters));
    }

    @Override
    public void periodic() {
        Time currentTime = RobotController.getMeasureTime();
        double deltaTime = currentTime.minus(lastTime).in(Seconds);

        // Note: ElevatorSim internally simulates gravity for vertical mechanisms (pitch = -90° for
        // upward, 90° for downward), matching the convention in LinearMechanism.java and
        // LinearConstants.java.
        // For non-vertical mechanisms, the gravity simulation is an approximation since
        // ElevatorSim doesn't support dynamic gravity angle changes.
        // The visualization and 3D pose correctly reflect the orientation, but physics
        // simulation uses the orientation set at construction time.
        // For accurate physics at arbitrary angles, consider using a custom LinearSystemSim.
        sim.setInputVoltage(inputs.appliedVoltage.in(Volts));
        sim.update(deltaTime);
        // Calculate the motor group's supply current 
        double supplyCurrentAmps = MotorSimCalcs.getSupplyCurrentAmps(RobotController.getBatteryVoltage(), inputs.appliedVoltage.in(Volts), 
        inputs.torqueCurrent.in(Amps)) * io.getNumberOfMotors();
        BatteryModel.addCurrentLoad(supplyCurrentAmps);

        AngularVelocity currentVelocity =
                toAngle(Meters.of(sim.getVelocityMetersPerSecond())).per(Seconds);
        AngularAcceleration currentAcceleration;
        if (deltaTime > 0) {
            currentAcceleration = currentVelocity.minus(lastVelocity).div(Seconds.of(deltaTime));
        } else {
            currentAcceleration = RadiansPerSecondPerSecond.zero();
        }

        lastTime = currentTime;
        lastVelocity = currentVelocity;

        io.setMechanismPosition(toAngle(Meters.of(sim.getPositionMeters())));
        io.setMechanismVelocity(currentVelocity);
        io.setMechanismAcceleration(currentAcceleration);

        super.periodic();
    }

    @Override
    public void setEncoderPosition(Angle position) {
        sim.setState(toDistance(position).in(Meters), sim.getVelocityMetersPerSecond());
    }
}
