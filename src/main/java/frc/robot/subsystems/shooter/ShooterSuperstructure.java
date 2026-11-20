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

package frc.robot.subsystems.shooter;

import static org.wpilib.units.Units.Rotations;

import frc.lib.actuators.flywheel.FlywheelActuator;
import frc.lib.actuators.rotary.RotaryActuator;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.util.PowerProfiler;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Power;

public class ShooterSuperstructure implements Mechanism, AutoCloseable {

    private final RotaryActuator<?, ?> hoodIO;
    private final FlywheelActuator<?> flywheelIO;

    /**
     * Constructs a new ShooterSuperstructure subsystem with the specified hood and flywheel
     * actuators.
     *
     * @param hoodIO the hood actuator for adjusting shot angle
     * @param flywheelIO the flywheel actuator for spinning up shots
     */
    public ShooterSuperstructure(RotaryActuator<?, ?> hoodIO, FlywheelActuator<?> flywheelIO) {
        this.hoodIO = hoodIO;
        this.flywheelIO = flywheelIO;

        Scheduler.getDefault().addPeriodic(this::periodic);
    }

    public void periodic() {
        flywheelIO.periodic();
        hoodIO.periodic();
    }

    private void setFlywheelVelocity(AngularVelocity velocity) {
        flywheelIO.runVelocity(velocity, FlywheelConstants.MAX_ACCELERATION, PIDSlot.SLOT_0);
    }

    private void setHoodPosition(Angle angle) {
        hoodIO.runUnprofiledPosition(angle, PIDSlot.SLOT_0);
    }

    /**
     * Gets the current angle of the hood.
     *
     * @return the hood's current position angle
     */
    public Angle getHoodAngle() {
        return hoodIO.getPosition();
    }

    public LinearVelocity getLinearVelocity() {
        return flywheelIO.getLinearVelocity();
    }

    /**
     * Return the current power draw of the flywheel actuator based on applied voltage and supply
     * current.
     */
    public Power getFlywheelPowerDraw() {
        return flywheelIO.getAppliedVoltage().times(flywheelIO.getSupplyCurrent());
    }

    /** Register the Shooter subsystem with the power profiler. */
    public void registerMechanisms(PowerProfiler powerProfiler) {
        powerProfiler.registerMechanism(getName() + "/Flywheel", flywheelIO);
        powerProfiler.registerMechanism(getName() + "/Hood", hoodIO);
    }

    /**
     * Creates a command to set the hood to a specific angle.
     *
     * @param angle the target angle for the hood
     * @return command that sets the hood angle
     */
    public Command setHoodAngle(Angle angle) {
        return this.run(c -> setHoodPosition(angle)).named("Set Hood Angle");
    }

    /**
     * Creates a command to set the flywheel velocity.
     *
     * @param velocity the target angular velocity for the flywheel
     * @return command that sets the flywheel speed
     */
    public Command setFlywheelSpeed(AngularVelocity velocity) {
        return this.run(c -> setFlywheelVelocity(velocity)).named("Set Flywheel Velocity");
    }

    public Command coastFlywheels() {
        return this.run(c -> flywheelIO.runCoast()).named("Coast Flywheels");
    }

    public Command stopAndStow() {
        return Command.sequence(setHoodAngle(Rotations.zero()), coastFlywheels())
                .named("Stop and Stow Shooter");
    }

    /**
     * A blocking command that lowers the hood and waits until it is zeroed. This should be included
     * with a timeout to compensate for possible sensor error. Primarily for use in autos.
     */
    public Command retractHood() {
        return setHoodAngle(Rotations.zero())
                .andThen(
                        Command.waitUntil(() -> hoodIO.getPosition().lte(HoodConstants.TOLERANCE))
                                .named("Wait for hood to be within tolerance"))
                .named("Retract Hood");
    }

    /** Closes all underlying actuators and releases resources. */
    @Override
    public void close() {
        flywheelIO.close();
        hoodIO.close();
    }
}
