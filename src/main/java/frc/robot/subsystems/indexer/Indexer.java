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
package frc.robot.subsystems.indexer;

import static org.wpilib.units.Units.Amps;

import frc.lib.actuators.flywheel.FlywheelActuator;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.PowerProfiler;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.units.measure.AngularVelocity;

/**
 * Subsystem that controls the indexer for moving game pieces within the robot. The indexer can run
 * forward to feed game pieces toward the shooter, run in reverse to eject them, or stop. Uses a
 * flywheel actuator and runs forward and reverse with torque-current control.
 */
public class Indexer implements Mechanism {
    private final FlywheelActuator<?> io;
    private boolean brownedOut = false;

    private static final LoggedTunableNumber SHOOT_TORQUE_CURRENT =
            new LoggedTunableNumber(IndexerConstants.NAME + "/ShootTorqueCurrent", 40.0);

    private static final LoggedTunableNumber EJECT_TORQUE_CURRENT =
            new LoggedTunableNumber(IndexerConstants.NAME + "/EjectTorqueCurrent", -40.0);

    /**
     * Constructs an Indexer subsystem.
     *
     * @param io The flywheel actuator for controlling the indexer motors
     */
    public Indexer(FlywheelActuator<?> io) {
        this.io = io;

        Scheduler.getDefault().addPeriodic(this::periodic);
    }

    public void periodic() {
        io.periodic();
    }

    private void stop() {
        io.runCoast();
    }

    /** Register the Indexer subsystem with the power profiler. */
    public void registerMechanisms(PowerProfiler powerProfiler) {
        powerProfiler.registerMechanism(getName(), io);
    }

    /**
     * Creates a command to stop the indexer by applying coast mode.
     *
     * @return a command that stops the indexer
     */
    public Command stopCommand() {
        return this.run(c -> this.stop()).named("Stop Indexer");
    }

    /**
     * Creates a command to run the indexer at shooting velocities. The indexer will stop when the
     * command is interrupted or cancelled.
     *
     * @return a command that runs the indexer at shooting speed
     */
    public Command shoot() {
        return this.run(
                        c -> {
                            io.runCurrent(Amps.of(SHOOT_TORQUE_CURRENT.get()));

                            c.park();
                        })
                .whenCanceled(this::stop)
                .named("Shoot");
    }

    /**
     * Creates a command to run the indexer in reverse to eject game pieces. The indexer will stop
     * when the command is interrupted or cancelled.
     *
     * @return a command that runs the indexer in reverse
     */
    public Command eject() {
        return this.run(
                        c -> {
                            io.runCurrent(Amps.of(EJECT_TORQUE_CURRENT.get()));

                            c.park();
                        })
                .whenCanceled(this::stop)
                .named("Eject");
    }

    public AngularVelocity getVelocity() {
        return io.getVelocity();
    }

    /**
     * Updates the indexer supply current limit for brownout protection.
     *
     * @param brownedOut True when the robot is actively browned out
     */
    public void setBrownedOut(boolean brownedOut) {
        if (this.brownedOut == brownedOut) {
            return;
        }
        this.brownedOut = brownedOut;
        io.setSupplyCurrentLimit(
                brownedOut
                        ? IndexerConstants.BROWNOUT_SUPPLY_CURRENT_LIMIT
                        : IndexerConstants.SUPPLY_CURRENT_LIMIT);
    }

    /** Toggles the indexer supply current limit for brownout protection. */
    public void toggleBrownedOut() {
        setBrownedOut(!brownedOut);
    }

    /** Closes the indexer actuator and releases resources. */
    public void close() {
        io.close();
    }
}
