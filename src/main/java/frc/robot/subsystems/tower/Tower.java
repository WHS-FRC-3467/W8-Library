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

package frc.robot.subsystems.tower;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.RotationsPerSecond;

import frc.lib.actuators.flywheel.FlywheelActuator;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.PowerProfiler;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

/**
 * Subsystem that controls the tower mechanism that transfers game pieces from the indexer to the
 * shooter. The tower can stop, idle at a slow speed to hold game pieces, or shoot at full speed.
 * Uses a flywheel mechanism for velocity control.
 */
public class Tower implements Mechanism {

    private static final LoggedTunableNumber SHOOT_TORQUE_CURRENT =
            new LoggedTunableNumber(TowerConstants.NAME + "/ShootTorqueCurrent", 40.0);

    private static final LoggedTunableNumber EJECT_TORQUE_CURRENT =
            new LoggedTunableNumber(TowerConstants.NAME + "/EjectTorqueCurrent", -40.0);

    private final FlywheelActuator<?> io;

    /**
     * Constructs a new Tower subsystem with the specified flywheel mechanism.
     *
     * @param io the flywheel mechanism IO implementation for the tower
     */
    public Tower(FlywheelActuator<?> io) {
        this.io = io;
    }

    public void periodic() {
        io.periodic();
    }

    private void stop() {
        io.runBrake();
    }

    /**
     * Checks if the tower is near its current velocity setpoint.
     *
     * @return true if the tower velocity is within tolerance of the setpoint
     */
    public boolean nearSetpoint() {
        return io.getVelocityError().lte(TowerConstants.TOLERANCE);
    }

    /**
     * Gets the current velocity of the tower motor.
     *
     * @return The velocity in rotations per second
     */
    public double getSpeed() {
        return io.getVelocity().in(RotationsPerSecond);
    }

    /** Register the Tower subsystem with the power profiler. */
    public void registerMechanisms(PowerProfiler powerProfiler) {
        powerProfiler.registerMechanism(getName(), io);
    }

    /**
     * Creates a command to run the tower at shooting velocity to transfer game pieces to the
     * shooter. The tower will stop when the command is interrupted or cancelled.
     *
     * @return a command that runs the tower at shooting speed
     */
    public Command shoot() {
        return this.run(c -> io.runCurrent(Amps.of(SHOOT_TORQUE_CURRENT.get())))
                .whenCanceled(this::stop)
                .named("Shoot");
    }

    /**
     * Creates a command to run the tower in reverse to eject game pieces. The tower will stop when
     * the command is interrupted or cancelled.
     *
     * @return a command that runs the tower in reverse
     */
    public Command eject() {
        return this.run(c -> io.runCurrent(Amps.of(EJECT_TORQUE_CURRENT.get())))
                .whenCanceled(this::stop)
                .named("Eject");
    }

    /**
     * Creates a command to stop the tower by applying brake mode.
     *
     * @return a command that stops the tower
     */
    public Command stopCommand() {
        return this.run(c -> io.runBrake()).whenCanceled(this::stop).named("Stop");
    }

    /** Closes the underlying flywheel mechanism and releases resources. */
    public void close() {
        io.close();
    }
}
