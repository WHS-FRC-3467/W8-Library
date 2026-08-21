// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import org.littletonrobotics.junction.Logger;

/**
 * A ProfiledPIDController with tunable gains and optional feedforward support.
 *
 * <p>This controller extends WPILib's ProfiledPIDController to add runtime tunability and
 * feedforward compensation. PID gains (kP, kI, kD) and motion profile constraints (max velocity and
 * acceleration) are required. Feedforward gains (kA, kV, kG, kS) and gravity compensation type are
 * optional and can be added using the builder methods.
 *
 * <p>Call {@link #updatePID()} periodically to check for and apply updated values from the
 * dashboard. The {@link #calculate} method automatically adds feedforward to the PID output.
 *
 * <p>Example usage:
 *
 * <pre>{@code
 * // Basic PID with motion profile
 * WindupPID pid = new WindupPID("Arm/PID", 2.0, 0.0, 0.1, 5.0, 10.0);
 *
 * // With feedforward and gravity compensation for an arm
 * WindupPID pid = new WindupPID("Arm/PID", 2.0, 0.0, 0.1, 5.0, 10.0)
 *     .withKV(0.5)
 *     .withKG(0.3)
 *     .withGravityType(GravityType.ARM);
 *
 * // In periodic():
 * pid.updatePID(); // Apply any dashboard changes
 * double output = pid.calculate(currentAngle, goalAngle);
 * }</pre>
 */
public class WindupPID extends ProfiledPIDController {

    /**
     * Describes how gravity affects the mechanism, which determines how the gravity feedforward
     * (kG) is applied in {@link #calculate}.
     */
    public enum GravityType {
        /** No gravity compensation. kG is ignored. */
        NONE,
        /** Constant gravity compensation for vertical elevators. Applies kG directly. */
        ELEVATOR,
        /** Angle-varying gravity compensation for rotating arms. Applies kG * cos(position). */
        ARM
    }

    // Tunable numbers for PID and motion profile (always present)
    private final LoggedTunableNumber p;
    private final LoggedTunableNumber i;
    private final LoggedTunableNumber d;
    private final LoggedTunableNumber maxVelocity;
    private final LoggedTunableNumber maxAcceleration;

    // Tunable numbers for feedforward gains (null if not configured)
    private LoggedTunableNumber kA = null;
    private LoggedTunableNumber kV = null;
    private LoggedTunableNumber kG = null;
    private LoggedTunableNumber kS = null;

    // Store the name prefix so builder methods can create LoggedTunableNumbers
    private final String name;

    private GravityType gravityType = GravityType.NONE;

    // Track the previous setpoint velocity to compute acceleration for kA feedforward
    private double previousVelocity = 0.0;

    /**
     * When enabled (toggled from the dashboard), each gain's contribution to the output is logged
     * individually to AdvantageKit under "{name}/GainContributions/".
     */
    private final LoggedTunableBoolean logGainContributions;

    /**
     * Constructs a WindupPID controller with required PID and motion profile gains.
     *
     * <p>Use builder methods like {@link #withKV}, {@link #withKG}, and {@link #withGravityType} to
     * optionally add feedforward compensation.
     *
     * @param name The logging key prefix for tunable values (e.g., "Arm/PID")
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @param maxV Maximum velocity for the motion profile
     * @param maxA Maximum acceleration for the motion profile
     */
    public WindupPID(String name, double kP, double kI, double kD, double maxV, double maxA) {
        super(kP, kI, kD, new TrapezoidProfile.Constraints(maxV, maxA));

        this.name = name;

        // Set up tunable numbers for PID and motion profile constants
        this.p = new LoggedTunableNumber(name + "/kP", kP);
        this.i = new LoggedTunableNumber(name + "/kI", kI);
        this.d = new LoggedTunableNumber(name + "/kD", kD);
        this.maxVelocity = new LoggedTunableNumber(name + "/maxVelocity", maxV);
        this.maxAcceleration = new LoggedTunableNumber(name + "/maxAcceleration", maxA);

        // Off by default — enable from the dashboard to see per-gain breakdowns in AdvantageScope
        this.logGainContributions = new LoggedTunableBoolean(name + "/LogGainContributions", false);
    }

    // -------------------------------------------------------------------------
    // Builder methods for optional feedforward gains
    // -------------------------------------------------------------------------

    /**
     * Adds an acceleration feedforward gain (kA). The feedforward voltage will include kA *
     * acceleration, where acceleration is estimated from the profile setpoint.
     *
     * @param defaultKA Default acceleration gain value
     * @return This controller, for method chaining
     */
    public WindupPID withKA(double defaultKA) {
        this.kA = new LoggedTunableNumber(name + "/kA", defaultKA);
        return this;
    }

    /**
     * Adds a velocity feedforward gain (kV). The feedforward voltage will include kV *
     * setpointVelocity.
     *
     * @param defaultKV Default velocity gain value
     * @return This controller, for method chaining
     */
    public WindupPID withKV(double defaultKV) {
        this.kV = new LoggedTunableNumber(name + "/kV", defaultKV);
        return this;
    }

    /**
     * Adds a gravity feedforward gain (kG). How it's applied depends on {@link #withGravityType}.
     * Set the gravity type with {@link #withGravityType} to get the correct behavior.
     *
     * @param defaultKG Default gravity gain value
     * @return This controller, for method chaining
     */
    public WindupPID withKG(double defaultKG) {
        this.kG = new LoggedTunableNumber(name + "/kG", defaultKG);
        return this;
    }

    /**
     * Adds a static friction feedforward gain (kS). The feedforward voltage will include kS *
     * sign(setpointVelocity) to overcome friction at the start of motion.
     *
     * @param defaultKS Default static friction gain value
     * @return This controller, for method chaining
     */
    public WindupPID withKS(double defaultKS) {
        this.kS = new LoggedTunableNumber(name + "/kS", defaultKS);
        return this;
    }

    /**
     * Sets the gravity compensation type. This controls how kG is applied in {@link #calculate}.
     * Has no effect if kG has not been set with {@link #withKG}.
     *
     * @param type The gravity compensation type
     * @return This controller, for method chaining
     */
    public WindupPID withGravityType(GravityType type) {
        this.gravityType = type;
        return this;
    }

    // -------------------------------------------------------------------------
    // Core methods
    // -------------------------------------------------------------------------

    /**
     * Updates PID gains and motion profile constraints from tunable values if they have changed.
     * Call this periodically (e.g., in a subsystem's {@code periodic()} method).
     */
    public void updatePID() {
        // Update PID gains if any have changed
        if (p.hasChanged(hashCode()) || i.hasChanged(hashCode()) || d.hasChanged(hashCode())) {
            this.setPID(p.get(), i.get(), d.get());
        }

        // Update motion profile constraints if max velocity or acceleration changed
        if (maxVelocity.hasChanged(hashCode()) || maxAcceleration.hasChanged(hashCode())) {
            this.setConstraints(
                    new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
        }
    }

    /**
     * Calculates the PID output plus feedforward for the given measurement and goal position.
     *
     * <p>The feedforward is computed from the motion profile setpoint and added to the PID output:
     *
     * <ul>
     *   <li>kS * sign(velocity) — overcomes static friction
     *   <li>kV * velocity — models velocity resistance
     *   <li>kA * acceleration — models inertia (estimated from profile)
     *   <li>kG component — gravity compensation (depends on {@link GravityType})
     * </ul>
     *
     * @param measurement The current measured position
     * @param goal The target position
     * @return Combined PID + feedforward output
     */
    @Override
    public double calculate(double measurement, double goal) {
        double pidOutput = super.calculate(measurement, goal);
        return pidOutput + calculateFeedforward(measurement, pidOutput);
    }

    /**
     * Calculates the PID output plus feedforward for the given measurement and goal state.
     *
     * @param measurement The current measured position
     * @param goal The target state (position and velocity)
     * @return Combined PID + feedforward output
     */
    @Override
    public double calculate(double measurement, TrapezoidProfile.State goal) {
        double pidOutput = super.calculate(measurement, goal);
        return pidOutput + calculateFeedforward(measurement, pidOutput);
    }

    /**
     * Calculates the feedforward component based on the current motion profile setpoint.
     *
     * @param position The current measured position (used for ARM gravity compensation)
     * @return The feedforward voltage to add to the PID output
     */
    private double calculateFeedforward(double position, double pidOutput) {
        TrapezoidProfile.State setpoint = getSetpoint();
        double velocity = setpoint.velocity;

        // Estimate acceleration from change in setpoint velocity over one loop cycle
        double acceleration = (velocity - previousVelocity) / getPeriod();
        previousVelocity = velocity;

        double ksContribution = 0.0;
        double kvContribution = 0.0;
        double kaContribution = 0.0;
        double kgContribution = 0.0;

        // Static friction: overcomes stiction at the start of motion
        if (kS != null) {
            ksContribution = kS.get() * Math.signum(velocity);
        }

        // Velocity feedforward: proportional to the desired velocity
        if (kV != null) {
            kvContribution = kV.get() * velocity;
        }

        // Acceleration feedforward: proportional to the estimated acceleration
        if (kA != null) {
            kaContribution = kA.get() * acceleration;
        }

        // Gravity feedforward: depends on the mechanism type
        if (kG != null) {
            switch (gravityType) {
                case ELEVATOR ->
                        // Constant force — gravity always pulls down by the same amount
                        kgContribution = kG.get();
                case ARM ->
                        // Varies with angle — gravity torque = kG * cos(angle)
                        kgContribution = kG.get() * Math.cos(position);
                default -> {
                    // GravityType.NONE: no gravity compensation
                }
            }
        }

        // When the dashboard toggle is on, log each gain's individual contribution so you can
        // inspect them in AdvantageScope under "{name}/GainContributions/"
        if (logGainContributions.get()) {
            String prefix = name + "/GainContributions/";
            double kpContribution = super.getP() * getPositionError();
            double kdContribution = super.getD() * getVelocityError();
            // Integral contribution is inferred: whatever the PID output can't be explained by P+D
            double kiContribution = pidOutput - kpContribution - kdContribution;
            Logger.recordOutput(prefix + "kP", kpContribution);
            Logger.recordOutput(prefix + "kI", kiContribution);
            Logger.recordOutput(prefix + "kD", kdContribution);
            Logger.recordOutput(prefix + "kS", ksContribution);
            Logger.recordOutput(prefix + "kV", kvContribution);
            Logger.recordOutput(prefix + "kA", kaContribution);
            Logger.recordOutput(prefix + "kG", kgContribution);
        }

        return ksContribution + kvContribution + kaContribution + kgContribution;
    }
}
