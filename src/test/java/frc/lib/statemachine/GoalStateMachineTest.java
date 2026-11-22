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

package frc.lib.statemachine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;

class GoalStateMachineTest {
    private enum Goal {
        STOP,
        RUN,
        DONE
    }

    /** A mechanism with no hardware. Each test gets a new one. */
    private static class TestMechanism implements Mechanism {}

    private final Scheduler scheduler = Scheduler.getDefault();
    private Mechanism mechanism;
    private GoalStateMachine<Goal> goals;

    /** Records every state entry, in order, so tests can check what the machine did. */
    private final List<Goal> entered = new ArrayList<>();

    private boolean runIsDone = false;

    @BeforeEach
    void setUp() {
        assertTrue(HAL.initialize());
        setEnabled(true);

        mechanism = new TestMechanism();
        goals = new GoalStateMachine<>("Test", mechanism, Goal.class, Goal.STOP);
        goals.addState(Goal.STOP, c -> entered.add(Goal.STOP));
        goals.addState(Goal.RUN, c -> entered.add(Goal.RUN));
        goals.addState(Goal.DONE, c -> entered.add(Goal.DONE));
        goals.addTransition(Goal.RUN, Goal.DONE, () -> runIsDone);
        goals.start();
    }

    @AfterEach
    void tearDown() {
        scheduler.cancelAll();
        setEnabled(false);
    }

    private static void setEnabled(boolean enabled) {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.notifyNewData();
    }

    private void runScheduler(int loops) {
        for (int i = 0; i < loops; i++) {
            scheduler.run();
        }
    }

    @Test
    void startsInInitialStateWhenEnabled() {
        assertFalse(goals.isRunning());

        runScheduler(1);

        assertTrue(goals.isRunning());
        assertEquals(Goal.STOP, goals.getActive());
        assertEquals(List.of(Goal.STOP), entered);
    }

    @Test
    void requestSwitchesState() {
        // Triggers read their value when the scheduler polls them, so make it before running.
        Trigger isRun = goals.is(Goal.RUN);
        runScheduler(1);
        assertFalse(isRun.getAsBoolean());

        goals.request(Goal.RUN);
        runScheduler(2);

        assertEquals(Goal.RUN, goals.getActive());
        assertEquals(List.of(Goal.STOP, Goal.RUN), entered);
        assertTrue(isRun.getAsBoolean());
    }

    @Test
    void stateHoldsWithoutRerunningItsBody() {
        runScheduler(10);

        assertEquals(List.of(Goal.STOP), entered);
    }

    @Test
    void requestingTheActiveStateDoesNothing() {
        runScheduler(1);

        goals.request(Goal.STOP);
        runScheduler(3);

        assertEquals(List.of(Goal.STOP), entered);
    }

    @Test
    void automaticTransitionFires() {
        runScheduler(1);
        goals.request(Goal.RUN);
        runScheduler(2);

        runIsDone = true;
        runScheduler(2);

        assertEquals(Goal.DONE, goals.getActive());
    }

    @Test
    void automaticTransitionFiresAgainWhenConditionStaysTrue() {
        // The condition never goes false between the two visits to RUN. A plain WPILib
        // StateMachine transition would not fire the second time.
        runIsDone = true;
        runScheduler(1);

        goals.request(Goal.RUN);
        runScheduler(3);
        assertEquals(Goal.DONE, goals.getActive());

        goals.request(Goal.STOP);
        runScheduler(3);
        assertEquals(Goal.STOP, goals.getActive());

        goals.request(Goal.RUN);
        runScheduler(3);
        assertEquals(Goal.DONE, goals.getActive());
        assertEquals(List.of(Goal.STOP, Goal.RUN, Goal.DONE, Goal.STOP, Goal.RUN, Goal.DONE), entered);
    }

    @Test
    void backToBackRequestsAreAllHandled() {
        runScheduler(1);

        goals.request(Goal.RUN);
        runScheduler(1);
        goals.request(Goal.STOP);
        runScheduler(1);
        goals.request(Goal.RUN);
        runScheduler(1);

        assertEquals(Goal.RUN, goals.getActive());
    }

    @Test
    void overrideCommandTakesMechanismThenMachineResumes() {
        Trigger isRun = goals.is(Goal.RUN);
        runScheduler(1);
        goals.request(Goal.RUN);
        runScheduler(2);
        assertTrue(isRun.getAsBoolean());

        Command override = mechanism.run(c -> c.park()).named("Override");
        scheduler.schedule(override);
        runScheduler(2);

        assertTrue(scheduler.isRunning(override));
        assertFalse(goals.isRunning());
        assertFalse(isRun.getAsBoolean());

        // A request made while the machine is not running is remembered.
        goals.request(Goal.DONE);
        scheduler.cancel(override);
        runScheduler(3);

        assertTrue(goals.isRunning());
        assertEquals(Goal.DONE, goals.getActive());
    }

    @Test
    void stopsWhenDisabledAndRestartsInInitialState() {
        runScheduler(1);
        goals.request(Goal.RUN);
        runScheduler(2);

        setEnabled(false);
        runScheduler(2);
        assertFalse(goals.isRunning());

        setEnabled(true);
        runScheduler(2);
        assertTrue(goals.isRunning());
        assertEquals(Goal.STOP, goals.getActive());
    }

    @Test
    void everyGoalNeedsAState() {
        GoalStateMachine<Goal> incomplete = new GoalStateMachine<>("Incomplete", mechanism, Goal.class, Goal.STOP);
        incomplete.addState(Goal.STOP, c -> {});

        assertThrows(IllegalStateException.class, incomplete::start);
    }
}
