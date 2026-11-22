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

import static org.wpilib.units.Units.Amps;

import frc.lib.actuators.flywheel.FlywheelActuator;
import frc.lib.statemachine.GoalStateMachine;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.PowerProfiler;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;

/** The roller on the end of the intake that pulls game pieces into the robot. */
public class IntakeRoller implements Mechanism, AutoCloseable {
    /** What the roller can be asked to do. */
    public enum Goal {
        /** Hold still. */
        STOP,
        /** Pull game pieces in. */
        INTAKE,
        /** Push game pieces back out. */
        EJECT
    }

    private static final LoggedTunableNumber INTAKE_TORQUE_CURRENT =
            new LoggedTunableNumber(IntakeRollerConstants.NAME + "/IntakeTorqueCurrent", 80.0);

    private static final LoggedTunableNumber EJECT_TORQUE_CURRENT =
            new LoggedTunableNumber(IntakeRollerConstants.NAME + "/EjectTorqueCurrent", -40.0);

    private final FlywheelActuator<?> io;
    private final GoalStateMachine<Goal> goals;

    /**
     * Constructs the intake roller.
     *
     * @param io the roller actuator
     */
    public IntakeRoller(FlywheelActuator<?> io) {
        this.io = io;

        goals = new GoalStateMachine<>(IntakeRollerConstants.NAME, this, Goal.class, Goal.STOP);
        goals.addState(Goal.STOP, c -> io.runBrake());
        goals.addState(Goal.INTAKE, c -> io.runCurrent(Amps.of(INTAKE_TORQUE_CURRENT.get())));
        goals.addState(Goal.EJECT, c -> io.runCurrent(Amps.of(EJECT_TORQUE_CURRENT.get())));
        goals.start();

        Scheduler.getDefault().addPeriodic(io::periodic);
    }

    /**
     * Asks the roller to switch to a goal.
     *
     * @param goal what the roller should do
     */
    public void request(Goal goal) {
        goals.request(goal);
    }

    /** Register the roller with the power profiler. */
    public void registerMechanisms(PowerProfiler powerProfiler) {
        powerProfiler.registerMechanism(IntakeRollerConstants.NAME, io);
    }

    @Override
    public void close() {
        io.close();
    }
}
