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

import static org.wpilib.units.Units.*;

import frc.lib.actuators.flywheel.FlywheelActuator;
import frc.lib.actuators.linear.LinearActuator;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.PowerProfiler;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.NeedsNameBuilderStage;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.*;

public class IntakeSuperstructure implements Mechanism, AutoCloseable {

    private final LinearActuator<?> intakeLinearIO;
    private final FlywheelActuator<?> intakeRollerIO;

    private final Trigger isExtended;
    private final Trigger isRetracted;
    private final Trigger isRollerSafe;

    private final Debouncer rollerSafeDebouncer = new Debouncer(0.02, Debouncer.DebounceType.RISING);

    private static final LoggedTunableNumber MIN_SAFE_ROLLER_DISTANCE =
            new LoggedTunableNumber(IntakeLinearConstants.NAME + "/MinSafeRollerDistanceInches", 5.0);

    public IntakeSuperstructure(LinearActuator<?> intakeLinearIO, FlywheelActuator<?> intakeRollerIO) {

        this.intakeLinearIO = intakeLinearIO;
        this.intakeRollerIO = intakeRollerIO;

        // Sets initial target position to fully retracted
        intakeLinearIO.runLinearPosition(
                IntakeLinearConstants.MIN_DISTANCE,
                PIDSlot.SLOT_0,
                IntakeLinearConstants.CRUISE_VELOCITY,
                IntakeLinearConstants.MAX_ACCELERATION);

        isExtended = new Trigger(() -> intakeLinearIO
                .getLinearPosition()
                .isNear(IntakeLinearConstants.MAX_DISTANCE, IntakeLinearConstants.TOLERANCE));

        isRetracted = new Trigger(() -> intakeLinearIO
                .getLinearPosition()
                .isNear(IntakeLinearConstants.MIN_DISTANCE, IntakeLinearConstants.TOLERANCE));

        isRollerSafe = new Trigger(() -> rollerSafeDebouncer.calculate(
                intakeLinearIO.getLinearPosition().gte(Inches.of(MIN_SAFE_ROLLER_DISTANCE.get()))));

        Scheduler.getDefault().addPeriodic(this::periodic);
    }

    /** Returns true if the intake roller is running and the intake is extended. */
    public boolean isIntaking() {
        return intakeRollerIO.getVelocity().in(RotationsPerSecond) > 1.0 && isExtended.getAsBoolean();
    }

    private Distance clampDistance(Distance goal) {
        return Meters.of(Math.clamp(
                goal.in(Meters),
                IntakeLinearConstants.MIN_DISTANCE.in(Meters),
                IntakeLinearConstants.MAX_DISTANCE.in(Meters)));
    }

    /** Register the Intake subsystem with the power profiler. */
    public void registerMechanisms(PowerProfiler powerProfiler) {
        powerProfiler.registerMechanism(getName() + "/IntakeLinear", intakeLinearIO);
        powerProfiler.registerMechanism(getName() + "/IntakeRoller", intakeRollerIO);
    }

    /**
     * Moves the intake to a goal distance using Motion Magic with the given cruise velocity and
     * acceleration. The command completes immediately after issuing the control request. Use {@link
     * Command#waitUntil} to block until the goal is reached.
     */
    private NeedsNameBuilderStage moveToPosition(
            Distance goal, LinearVelocity cruiseVelocity, LinearAcceleration acceleration) {
        Distance clampedGoal = clampDistance(goal);

        return this.run(
                c -> intakeLinearIO.runLinearPosition(clampedGoal, PIDSlot.SLOT_0, cruiseVelocity, acceleration));
    }

    private Command runRoller() {
        return this.run(c -> intakeRollerIO.runCurrent(Amps.of(80.0))).named("Run Roller");
    }

    public Command stopRoller() {
        return this.run(c -> intakeRollerIO.runBrake()).named("Stop Roller");
    }

    public Command ejectRoller() {
        return this.run(c -> intakeRollerIO.runCurrent(Amps.of(-40))).named("Eject Roller");
    }

    public Command intake() {
        return this.run(c -> {
                    Command extend = moveToPosition(
                                    IntakeLinearConstants.MAX_DISTANCE,
                                    IntakeLinearConstants.CRUISE_VELOCITY,
                                    IntakeLinearConstants.MAX_ACCELERATION)
                            .named("Extend Intake");

                    c.await(extend);
                    c.waitUntil(isRollerSafe);
                    c.await(runRoller());
                    c.waitUntil(isExtended);
                })
                .named("Extend Intake With Roller");
    }

    public Command retractIntake() {
        return this.run(c -> {
                    Command retract = moveToPosition(
                                    IntakeLinearConstants.MIN_DISTANCE,
                                    IntakeLinearConstants.CRUISE_VELOCITY,
                                    IntakeLinearConstants.MAX_ACCELERATION)
                            .named("Retract Linear");

                    c.await(retract);
                    if (isRollerSafe.getAsBoolean()) {
                        c.await(runRoller());
                        c.waitUntil(isRollerSafe.negate());
                        c.await(stopRoller());
                    }
                    c.waitUntil(isRetracted);
                })
                .named("Retract Intake With Roller");
    }

    public Command linearCoast() {
        return this.run(c -> intakeLinearIO.runCoast()).named("Linear Coast");
    }

    public Command homeLinear() {
        return this.run(c -> {
                    intakeLinearIO.runDutyCycle(0.25, true);

                    c.park();
                })
                .whenCanceled(() -> intakeLinearIO.setEncoderPosition(Rotations.of(3.7)))
                .named("Home Linear");
    }

    public void periodic() {
        intakeLinearIO.periodic();
        intakeRollerIO.periodic();
    }

    /**
     * Gets the linear extension of the subsystem by converting the motor's rotation.
     *
     * @return The estimated linear extension of the subsystem
     */
    public Distance getExtension() {
        return intakeLinearIO.getLinearPosition();
    }

    @Override
    public void close() {
        intakeRollerIO.close();
        intakeLinearIO.close();
    }
}
