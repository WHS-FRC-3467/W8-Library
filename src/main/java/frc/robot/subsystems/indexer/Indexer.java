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
import frc.lib.statemachine.GoalStateMachine;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.PowerProfiler;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.units.measure.AngularVelocity;

/**
 * The indexer, which moves game pieces from the intake toward the tower. It runs with
 * torque-current control.
 */
public class Indexer implements Mechanism, AutoCloseable {
    /** What the indexer can be asked to do. */
    public enum Goal {
        /** Coast with no power applied. */
        STOP,
        /** Push game pieces toward the tower. */
        FEED,
        /** Push game pieces back toward the intake. */
        EJECT
    }

    private static final LoggedTunableNumber FEED_TORQUE_CURRENT =
            new LoggedTunableNumber(IndexerConstants.NAME + "/FeedTorqueCurrent", 40.0);

    private static final LoggedTunableNumber EJECT_TORQUE_CURRENT =
            new LoggedTunableNumber(IndexerConstants.NAME + "/EjectTorqueCurrent", -40.0);

    private final FlywheelActuator<?> io;
    private final GoalStateMachine<Goal> goals;
    private boolean brownedOut = false;

    /**
     * Constructs the indexer.
     *
     * @param io the indexer actuator
     */
    public Indexer(FlywheelActuator<?> io) {
        this.io = io;

        goals = new GoalStateMachine<>(IndexerConstants.NAME, this, Goal.class, Goal.STOP);
        goals.addState(Goal.STOP, c -> io.runCoast());
        goals.addState(Goal.FEED, c -> io.runCurrent(Amps.of(FEED_TORQUE_CURRENT.get())));
        goals.addState(Goal.EJECT, c -> io.runCurrent(Amps.of(EJECT_TORQUE_CURRENT.get())));
        goals.start();

        Scheduler.getDefault().addPeriodic(io::periodic);
    }

    /**
     * Asks the indexer to switch to a goal.
     *
     * @param goal what the indexer should do
     */
    public void request(Goal goal) {
        goals.request(goal);
    }

    public AngularVelocity getVelocity() {
        return io.getVelocity();
    }

    /** Register the indexer with the power profiler. */
    public void registerMechanisms(PowerProfiler powerProfiler) {
        powerProfiler.registerMechanism(IndexerConstants.NAME, io);
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
                brownedOut ? IndexerConstants.BROWNOUT_SUPPLY_CURRENT_LIMIT : IndexerConstants.SUPPLY_CURRENT_LIMIT);
    }

    @Override
    public void close() {
        io.close();
    }
}
