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

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.StateMachine.State;
import org.wpilib.command3.Trigger;

/**
 * A state machine that always runs on one {@link Mechanism} and moves between states named by an
 * enum. This is the building block for our "goal state" control structure:
 *
 * <ul>
 *   <li>Each enum value is one state. A state is a small piece of coroutine code that tells the
 *       hardware what to do, then holds until the machine leaves the state.
 *   <li>Other code never commands the hardware directly. It calls {@link #request(Enum)} and the
 *       machine switches to that state on the next loop.
 *   <li>States can also switch on their own with {@link #addTransition}, for example "go from
 *       AIM to SHOOT once the flywheel is at speed".
 * </ul>
 *
 * <p>The machine is installed as the owning mechanism's default command by {@link #start()}. That
 * gives it the lifecycle we want for free: it stops when the robot is disabled, starts again from
 * the initial state when the robot is enabled, and steps aside if a normal command (homing, SysId,
 * a manual override) takes the mechanism, then resumes when that command ends.
 *
 * <pre>{@code
 * public class Roller implements Mechanism {
 *     public enum Goal { STOP, INTAKE }
 *
 *     private final GoalStateMachine<Goal> goals =
 *             new GoalStateMachine<>("Roller", this, Goal.class, Goal.STOP);
 *
 *     public Roller(FlywheelActuator<?> io) {
 *         goals.addState(Goal.STOP, c -> io.runBrake());
 *         goals.addState(Goal.INTAKE, c -> io.runCurrent(Amps.of(40)));
 *         goals.start();
 *     }
 *
 *     public void request(Goal goal) {
 *         goals.request(goal);
 *     }
 * }
 * }</pre>
 *
 * @param <G> the enum that names this machine's states
 */
public class GoalStateMachine<G extends Enum<G>> {
    /** A state-to-state transition that was declared before {@link #start()} built the machine. */
    private record PendingTransition<G>(G from, G to, BooleanSupplier condition) {}

    private final String name;
    private final Mechanism owner;
    private final Class<G> goalType;
    private final G initialGoal;

    private final Map<G, Consumer<Coroutine>> bodies;
    private final List<PendingTransition<G>> pendingTransitions = new ArrayList<>();

    private Command defaultCommand = null;

    /** The state the machine is in right now. */
    private G active;

    /** The state somebody asked for that the machine has not switched to yet, or null. */
    private G pendingRequest = null;

    /** The most recent request, kept only for logging. */
    private G lastRequested;

    /** Counts state entries, so each transition condition can tell when a state was (re)entered. */
    private int entryCount = 0;

    /**
     * Creates a goal state machine. Add states with {@link #addState}, then call {@link #start()}.
     *
     * @param name name used for logging and for the commands this machine creates
     * @param owner the mechanism the states control. Every state's command requires it.
     * @param goalType the enum class that names the states
     * @param initialGoal the state to start in every time the machine starts (on enable, or after
     *     another command gives the mechanism back)
     */
    public GoalStateMachine(String name, Mechanism owner, Class<G> goalType, G initialGoal) {
        this.name = name;
        this.owner = owner;
        this.goalType = goalType;
        this.initialGoal = initialGoal;
        this.bodies = new EnumMap<>(goalType);
        this.active = initialGoal;
        this.lastRequested = initialGoal;
    }

    /**
     * Defines what the mechanism does in one state.
     *
     * <p>The body runs once each time the state is entered. It can be a single hardware call, or a
     * short sequence using the coroutine ({@code c.waitUntil(...)}, {@code c.wait(...)}). When the
     * body returns, the state keeps holding until a request or transition moves the machine on, so
     * a body never needs to end with {@code c.park()}. If the machine leaves the state while the
     * body is still waiting, the body is canceled at that wait.
     *
     * @param goal the state being defined
     * @param body the code to run on entering the state
     */
    public void addState(G goal, Consumer<Coroutine> body) {
        requireNotStarted();
        if (bodies.containsKey(goal)) {
            throw new IllegalArgumentException(name + " already has a state for " + goal);
        }
        bodies.put(goal, body);
    }

    /**
     * Makes the machine switch from one state to another by itself when a condition is true.
     * Automatic transitions are checked before requests, in the order they were added.
     *
     * @param from the state the machine must be in
     * @param to the state to switch to
     * @param condition checked every loop while in {@code from}
     */
    public void addTransition(G from, G to, BooleanSupplier condition) {
        requireNotStarted();
        pendingTransitions.add(new PendingTransition<>(from, to, condition));
    }

    /**
     * Builds the state machine and installs it as the owner's default command. Call this once, at
     * the end of the mechanism's constructor, after every state and transition has been added.
     */
    public void start() {
        requireNotStarted();
        for (G goal : goalType.getEnumConstants()) {
            if (!bodies.containsKey(goal)) {
                throw new IllegalStateException(name + " has no state for " + goal + ". Add one with addState().");
            }
        }

        StateMachine machine = new StateMachine(name + " States");
        Map<G, State> states = new EnumMap<>(goalType);

        for (G goal : goalType.getEnumConstants()) {
            Consumer<Coroutine> body = bodies.get(goal);
            State state = machine.addState(owner.run(c -> {
                        body.accept(c);
                        // Hold the mechanism until the machine leaves this state. A state whose
                        // command finishes by itself would end the whole state machine.
                        c.park();
                    })
                    .named(name + "/" + goal));
            state.onEnter(() -> enter(goal));
            states.put(goal, state);
        }
        machine.setInitialState(states.get(initialGoal));

        // Automatic transitions first, so they win over a request made in the same loop.
        for (PendingTransition<G> transition : pendingTransitions) {
            states.get(transition.from())
                    .switchTo(states.get(transition.to()))
                    .when(afterFirstCheck(transition.condition()));
        }

        // One transition shared by every state: go wherever the pending request points.
        machine.switchFromAny()
                .to(() -> states.get(pendingRequest))
                .when(afterFirstCheck(() -> pendingRequest != null));

        // The WPILib state machine has no requirements of its own, so it can't be a default
        // command. This wrapper requires the owner and simply runs the machine as its child.
        defaultCommand = owner.run(c -> c.await(machine))
                .withPriority(Command.LOWEST_PRIORITY)
                .named(name + " Goals");
        owner.setDefaultCommand(defaultCommand);

        log();
    }

    /**
     * Asks the machine to switch to a state. The switch happens the next time the scheduler runs.
     * Requesting the state the machine is already in does nothing (and drops any earlier request
     * that has not been handled yet).
     *
     * <p>If the machine is not running (robot disabled, or another command has the mechanism), the
     * request is remembered and applied as soon as the machine runs again.
     *
     * @param goal the state to switch to
     */
    public void request(G goal) {
        lastRequested = goal;
        pendingRequest = (goal == active && isRunning()) ? null : goal;
        log();
    }

    /**
     * Makes a command that requests a state and finishes immediately. Use this for button bindings
     * and inside autos.
     *
     * @param goal the state to request
     * @return a command that calls {@link #request(Enum)} once
     */
    public Command requestCommand(G goal) {
        return Command.noRequirements(c -> request(goal)).named(name + " Request " + goal);
    }

    /**
     * Gets the state the machine is in. While the machine is not running this is the last state it
     * was in.
     *
     * @return the active state
     */
    public G getActive() {
        return active;
    }

    /**
     * Makes a trigger that is true while the machine is running and in the given state.
     *
     * @param goal the state to check for
     * @return a trigger for that state
     */
    public Trigger is(G goal) {
        return new Trigger(() -> active == goal && isRunning());
    }

    /**
     * Checks whether the machine currently controls its mechanism. False while disabled and while
     * another command has taken the mechanism.
     *
     * @return true if the machine is running
     */
    public boolean isRunning() {
        return defaultCommand != null && owner.getRegisteredScheduler().isRunning(defaultCommand);
    }

    /** Runs when a state is entered. */
    private void enter(G goal) {
        active = goal;
        entryCount++;
        if (pendingRequest == goal) {
            pendingRequest = null;
        }
        log();
    }

    /**
     * Wraps a transition condition so it always reads false the first time it is checked after a
     * state is entered.
     *
     * <p>WPILib's StateMachine only fires a transition on the rising edge of its condition, and it
     * remembers the last value it saw even while the machine is in other states. Without this
     * wrapper, a condition that fired once and is still true the next time its state is entered
     * (for example "flywheel at speed" when re-entering AIM right after a shot) would never fire
     * again. Forcing one false reading on entry resets that memory. The cost is that every state
     * lasts at least one loop.
     */
    private BooleanSupplier afterFirstCheck(BooleanSupplier condition) {
        return new BooleanSupplier() {
            private int lastEntrySeen = -1;

            @Override
            public boolean getAsBoolean() {
                if (lastEntrySeen != entryCount) {
                    lastEntrySeen = entryCount;
                    return false;
                }
                return condition.getAsBoolean();
            }
        };
    }

    private void requireNotStarted() {
        if (defaultCommand != null) {
            throw new IllegalStateException(name + " has already been started");
        }
    }

    private void log() {
        Logger.recordOutput(name + "/ActiveState", active.name());
        Logger.recordOutput(name + "/RequestedState", lastRequested.name());
    }
}
