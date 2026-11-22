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

import static org.wpilib.units.Units.RotationsPerSecond;

import frc.lib.actuators.flywheel.FlywheelActuator;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.statemachine.GoalStateMachine;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.PowerProfiler;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.units.measure.AngularVelocity;

/** The shooter flywheel. It either coasts or holds the shooting speed. */
public class Flywheel implements Mechanism, AutoCloseable {
    /** What the flywheel can be asked to do. */
    public enum Goal {
        /** Coast with no power applied. */
        IDLE,
        /** Spin up to the shooting speed and hold it. */
        SHOOT
    }

    private static final LoggedTunableNumber SHOOT_SPEED_RPS =
            new LoggedTunableNumber(FlywheelConstants.NAME + "/ShootSpeedRPS", 50.0);

    private final FlywheelActuator<?> io;
    private final GoalStateMachine<Goal> goals;

    /** True while the flywheel is being asked to shoot and has reached the shooting speed. */
    public final Trigger atShootSpeed;

    /**
     * Constructs the flywheel.
     *
     * @param io the flywheel actuator
     */
    public Flywheel(FlywheelActuator<?> io) {
        this.io = io;

        goals = new GoalStateMachine<>(FlywheelConstants.NAME, this, Goal.class, Goal.IDLE);
        goals.addState(Goal.IDLE, c -> io.runCoast());
        goals.addState(
                Goal.SHOOT, c -> io.runVelocity(getShootSpeed(), FlywheelConstants.MAX_ACCELERATION, PIDSlot.SLOT_0));
        goals.start();

        atShootSpeed =
                goals.is(Goal.SHOOT).and(() -> io.getVelocity().isNear(getShootSpeed(), FlywheelConstants.TOLERANCE));

        Scheduler.getDefault().addPeriodic(io::periodic);
    }

    /**
     * Asks the flywheel to switch to a goal.
     *
     * @param goal what the flywheel should do
     */
    public void request(Goal goal) {
        goals.request(goal);
    }

    /**
     * Gets how fast the flywheel is spinning.
     *
     * @return the flywheel velocity
     */
    public AngularVelocity getVelocity() {
        return io.getVelocity();
    }

    private static AngularVelocity getShootSpeed() {
        return RotationsPerSecond.of(SHOOT_SPEED_RPS.get());
    }

    /** Register the flywheel with the power profiler. */
    public void registerMechanisms(PowerProfiler powerProfiler) {
        powerProfiler.registerMechanism(FlywheelConstants.NAME, io);
    }

    @Override
    public void close() {
        io.close();
    }
}
