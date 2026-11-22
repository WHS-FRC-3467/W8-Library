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

import static org.wpilib.units.Units.Degrees;

import frc.lib.actuators.rotary.RotaryActuator;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.statemachine.GoalStateMachine;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.PowerProfiler;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.units.measure.Angle;

/** The shooter hood, which sets the angle the game piece leaves at. */
public class Hood implements Mechanism, AutoCloseable {
    /** What the hood can be asked to do. */
    public enum Goal {
        /** Fully lowered. */
        STOW,
        /** Raised to the shooting angle. */
        SHOOT
    }

    private static final LoggedTunableNumber SHOOT_ANGLE_DEGREES =
            new LoggedTunableNumber(HoodConstants.NAME + "/ShootAngleDegrees", 15.0);

    private final RotaryActuator<?, ?> io;
    private final GoalStateMachine<Goal> goals;

    /** True while the hood is being asked to shoot and has reached the shooting angle. */
    public final Trigger atShootAngle;

    /**
     * Constructs the hood.
     *
     * @param io the hood actuator
     */
    public Hood(RotaryActuator<?, ?> io) {
        this.io = io;

        goals = new GoalStateMachine<>(HoodConstants.NAME, this, Goal.class, Goal.STOW);
        goals.addState(Goal.STOW, c -> io.runUnprofiledPosition(HoodConstants.MIN_ANGLE, PIDSlot.SLOT_0));
        goals.addState(Goal.SHOOT, c -> io.runUnprofiledPosition(getShootAngle(), PIDSlot.SLOT_0));
        goals.start();

        atShootAngle = goals.is(Goal.SHOOT).and(() -> io.nearGoal(getShootAngle(), HoodConstants.TOLERANCE));

        Scheduler.getDefault().addPeriodic(io::periodic);
    }

    /**
     * Asks the hood to switch to a goal.
     *
     * @param goal what the hood should do
     */
    public void request(Goal goal) {
        goals.request(goal);
    }

    /**
     * Gets the current angle of the hood.
     *
     * @return the hood angle
     */
    public Angle getAngle() {
        return io.getPosition();
    }

    private static Angle getShootAngle() {
        return Degrees.of(SHOOT_ANGLE_DEGREES.get());
    }

    /** Register the hood with the power profiler. */
    public void registerMechanisms(PowerProfiler powerProfiler) {
        powerProfiler.registerMechanism(HoodConstants.NAME, io);
    }

    @Override
    public void close() {
        io.close();
    }
}
