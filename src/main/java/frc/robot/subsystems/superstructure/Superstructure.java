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

package frc.robot.subsystems.superstructure;

import frc.lib.statemachine.GoalStateMachine;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.intake.IntakeSlide;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.tower.Tower;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Trigger;

/**
 * The robot-level state machine. It owns no hardware. Each of its states tells every mechanism
 * which goal to be in, so the whole robot is always in exactly one named state.
 *
 * <p>Drivers and autos only ever talk to this class, through the command methods at the bottom.
 * They never request mechanism goals directly.
 *
 * <p>The drivetrain is not part of this state machine. It keeps its own commands.
 */
public class Superstructure implements Mechanism {
    /** The states the robot can be in. */
    public enum Goal {
        /** Everything stopped and stowed. */
        IDLE,
        /** Intake out and collecting. */
        INTAKE,
        /** Flywheel spinning up and hood rising. Switches to SHOOT by itself once both are ready. */
        AIM,
        /** Feeding game pieces into the spinning flywheel. */
        SHOOT,
        /** Everything running backwards to clear a jam. */
        EJECT
    }

    private final GoalStateMachine<Goal> goals;

    /** True when the flywheel and hood are both ready for a shot. */
    public final Trigger readyToShoot;

    /** True while game pieces are being fed into the flywheel. */
    public final Trigger isShooting;

    /**
     * Constructs the superstructure.
     *
     * @param slide the intake slide
     * @param roller the intake roller
     * @param indexer the indexer
     * @param tower the tower
     * @param flywheel the shooter flywheel
     * @param hood the shooter hood
     */
    public Superstructure(
            IntakeSlide slide, IntakeRoller roller, Indexer indexer, Tower tower, Flywheel flywheel, Hood hood) {
        readyToShoot = flywheel.atShootSpeed.and(hood.atShootAngle);

        goals = new GoalStateMachine<>("Superstructure", this, Goal.class, Goal.IDLE);

        goals.addState(Goal.IDLE, c -> {
            indexer.request(Indexer.Goal.STOP);
            tower.request(Tower.Goal.STOP);
            flywheel.request(Flywheel.Goal.IDLE);
            hood.request(Hood.Goal.STOW);

            // Keep the roller turning while the intake is pulled in so game pieces aren't dropped,
            // but stop it before it gets close enough to hit the robot.
            slide.request(IntakeSlide.Goal.RETRACT);
            if (slide.isRollerSafe.getAsBoolean()) {
                roller.request(IntakeRoller.Goal.INTAKE);
                c.waitUntil(slide.isRollerSafe.negate());
            }
            roller.request(IntakeRoller.Goal.STOP);
        });

        goals.addState(Goal.INTAKE, c -> {
            indexer.request(Indexer.Goal.STOP);
            tower.request(Tower.Goal.STOP);
            flywheel.request(Flywheel.Goal.IDLE);
            hood.request(Hood.Goal.STOW);

            // Don't spin the roller until the slide has carried it clear of the robot.
            slide.request(IntakeSlide.Goal.EXTEND);
            roller.request(IntakeRoller.Goal.STOP);
            c.waitUntil(slide.isRollerSafe);
            roller.request(IntakeRoller.Goal.INTAKE);
        });

        goals.addState(Goal.AIM, c -> {
            slide.request(IntakeSlide.Goal.RETRACT);
            roller.request(IntakeRoller.Goal.STOP);
            indexer.request(Indexer.Goal.STOP);
            tower.request(Tower.Goal.STOP);
            flywheel.request(Flywheel.Goal.SHOOT);
            hood.request(Hood.Goal.SHOOT);
        });

        goals.addState(Goal.SHOOT, c -> {
            slide.request(IntakeSlide.Goal.RETRACT);
            roller.request(IntakeRoller.Goal.STOP);
            flywheel.request(Flywheel.Goal.SHOOT);
            hood.request(Hood.Goal.SHOOT);
            indexer.request(Indexer.Goal.FEED);
            tower.request(Tower.Goal.FEED);
        });

        goals.addState(Goal.EJECT, c -> {
            slide.request(IntakeSlide.Goal.EXTEND);
            flywheel.request(Flywheel.Goal.IDLE);
            hood.request(Hood.Goal.STOW);
            indexer.request(Indexer.Goal.EJECT);
            tower.request(Tower.Goal.EJECT);
            roller.request(IntakeRoller.Goal.STOP);
            c.waitUntil(slide.isRollerSafe);
            roller.request(IntakeRoller.Goal.EJECT);
        });

        // The only way into SHOOT: the robot must be aiming and the shooter must be ready.
        goals.addTransition(Goal.AIM, Goal.SHOOT, readyToShoot);

        goals.start();

        isShooting = goals.is(Goal.SHOOT);
    }

    /**
     * Gets the state the robot is in.
     *
     * @return the active state
     */
    public Goal getState() {
        return goals.getActive();
    }

    /**
     * Creates a command that stops and stows everything.
     *
     * @return a command that finishes immediately
     */
    public Command stow() {
        return goals.requestCommand(Goal.IDLE);
    }

    /**
     * Creates a command that starts intaking.
     *
     * @return a command that finishes immediately
     */
    public Command intake() {
        return goals.requestCommand(Goal.INTAKE);
    }

    /**
     * Creates a command that starts a shot. The robot aims first, then shoots by itself once the
     * shooter is ready, and keeps shooting until another state is requested.
     *
     * @return a command that finishes immediately
     */
    public Command score() {
        return goals.requestCommand(Goal.AIM);
    }

    /**
     * Creates a command that starts ejecting.
     *
     * @return a command that finishes immediately
     */
    public Command eject() {
        return goals.requestCommand(Goal.EJECT);
    }
}
