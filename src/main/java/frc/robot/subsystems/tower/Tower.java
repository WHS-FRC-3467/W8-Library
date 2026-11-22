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
import frc.lib.statemachine.GoalStateMachine;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.PowerProfiler;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;

/**
 * The tower, which lifts game pieces from the indexer into the shooter. It runs with
 * torque-current control.
 */
public class Tower implements Mechanism, AutoCloseable {
    /** What the tower can be asked to do. */
    public enum Goal {
        /** Hold still so game pieces don't creep into the flywheel. */
        STOP,
        /** Push game pieces up into the shooter. */
        FEED,
        /** Push game pieces back down toward the indexer. */
        EJECT
    }

    private static final LoggedTunableNumber FEED_TORQUE_CURRENT =
            new LoggedTunableNumber(TowerConstants.NAME + "/FeedTorqueCurrent", 40.0);

    private static final LoggedTunableNumber EJECT_TORQUE_CURRENT =
            new LoggedTunableNumber(TowerConstants.NAME + "/EjectTorqueCurrent", -40.0);

    private final FlywheelActuator<?> io;
    private final GoalStateMachine<Goal> goals;

    /**
     * Constructs the tower.
     *
     * @param io the tower actuator
     */
    public Tower(FlywheelActuator<?> io) {
        this.io = io;

        goals = new GoalStateMachine<>(TowerConstants.NAME, this, Goal.class, Goal.STOP);
        goals.addState(Goal.STOP, c -> io.runBrake());
        goals.addState(Goal.FEED, c -> io.runCurrent(Amps.of(FEED_TORQUE_CURRENT.get())));
        goals.addState(Goal.EJECT, c -> io.runCurrent(Amps.of(EJECT_TORQUE_CURRENT.get())));
        goals.start();

        Scheduler.getDefault().addPeriodic(io::periodic);
    }

    /**
     * Asks the tower to switch to a goal.
     *
     * @param goal what the tower should do
     */
    public void request(Goal goal) {
        goals.request(goal);
    }

    /**
     * Gets the current velocity of the tower motor.
     *
     * @return The velocity in rotations per second
     */
    public double getSpeed() {
        return io.getVelocity().in(RotationsPerSecond);
    }

    /** Register the tower with the power profiler. */
    public void registerMechanisms(PowerProfiler powerProfiler) {
        powerProfiler.registerMechanism(TowerConstants.NAME, io);
    }

    @Override
    public void close() {
        io.close();
    }
}
