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

package frc.robot.subsystems.intake;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Rotations;

import frc.lib.actuators.linear.LinearActuator;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.statemachine.GoalStateMachine;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.PowerProfiler;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.units.measure.Distance;

/** The linear slide that pushes the intake out past the bumpers and pulls it back in. */
public class IntakeSlide implements Mechanism, AutoCloseable {
    /** What the slide can be asked to do. */
    public enum Goal {
        /** Pulled fully inside the robot. */
        RETRACT,
        /** Pushed fully out to collect game pieces. */
        EXTEND
    }

    private static final LoggedTunableNumber MIN_SAFE_ROLLER_DISTANCE =
            new LoggedTunableNumber(IntakeLinearConstants.NAME + "/MinSafeRollerDistanceInches", 5.0);

    private final LinearActuator<?> io;
    private final GoalStateMachine<Goal> goals;

    /** True when the slide is far enough out that the roller can spin without hitting the robot. */
    public final Trigger isRollerSafe;

    /** True when the slide has reached the fully extended position. */
    public final Trigger isExtended;

    /** True when the slide has reached the fully retracted position. */
    public final Trigger isRetracted;

    /**
     * Constructs the intake slide.
     *
     * @param io the slide actuator
     */
    public IntakeSlide(LinearActuator<?> io) {
        this.io = io;

        goals = new GoalStateMachine<>(IntakeLinearConstants.NAME, this, Goal.class, Goal.RETRACT);
        goals.addState(Goal.RETRACT, c -> moveTo(IntakeLinearConstants.MIN_DISTANCE));
        goals.addState(Goal.EXTEND, c -> moveTo(IntakeLinearConstants.MAX_DISTANCE));
        goals.start();

        isRollerSafe = new Trigger(() -> io.getLinearPosition().gte(Inches.of(MIN_SAFE_ROLLER_DISTANCE.get())));
        isExtended =
                new Trigger(() -> io.nearGoal(IntakeLinearConstants.MAX_DISTANCE, IntakeLinearConstants.TOLERANCE));
        isRetracted =
                new Trigger(() -> io.nearGoal(IntakeLinearConstants.MIN_DISTANCE, IntakeLinearConstants.TOLERANCE));

        Scheduler.getDefault().addPeriodic(io::periodic);
    }

    private void moveTo(Distance position) {
        io.runLinearPosition(
                position,
                PIDSlot.SLOT_0,
                IntakeLinearConstants.CRUISE_VELOCITY,
                IntakeLinearConstants.MAX_ACCELERATION);
    }

    /**
     * Asks the slide to switch to a goal.
     *
     * @param goal what the slide should do
     */
    public void request(Goal goal) {
        goals.request(goal);
    }

    /**
     * Creates a command that drives the slide outward against its hard stop for as long as the
     * command runs, then resets the encoder to the hard stop position when it ends.
     *
     * <p>This is an example of an override command. It is a normal command that requires the
     * slide, so while it runs it takes the slide away from the goal state machine. When it ends,
     * the state machine starts again by itself.
     *
     * @return the homing command
     */
    public Command home() {
        return run(c -> {
                    io.runDutyCycle(0.25, true);
                    c.park();
                })
                .whenCanceled(() -> io.setEncoderPosition(Rotations.of(3.7)))
                .named("Home Intake Slide");
    }

    /** Register the slide with the power profiler. */
    public void registerMechanisms(PowerProfiler powerProfiler) {
        powerProfiler.registerMechanism(IntakeLinearConstants.NAME, io);
    }

    @Override
    public void close() {
        io.close();
    }
}
