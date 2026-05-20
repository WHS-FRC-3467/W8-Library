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

package frc.lib.mechanisms.differential;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMotorConstants;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import frc.lib.io.absoluteencoder.AbsoluteEncoderIO;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.differential.DifferentialMechanism.DiffMechCharacteristics;

import org.littletonrobotics.junction.Logger;

import java.util.Optional;

/**
 * A real-hardware implementation of {@link DifferentialMechanism} using CTRE's {@link
 * com.ctre.phoenix6.mechanisms.DifferentialMechanism} class.
 *
 * <p>This class wraps CTRE's mechanism for all control operations (position, velocity, voltage) and
 * reads back status signals for logging. The CTRE mechanism handles the internal synchronization
 * between the two TalonFX motors and provides automatic fault detection via its {@code periodic()}
 * method.
 *
 * <p><b>Requirements</b>: Phoenix Pro license and a CAN FD bus (CANivore). See the <a
 * href="https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/mechanisms/differential/differential-setup.html">CTRE
 * setup guide</a> for configuration details.
 *
 * <p>Usage example:
 *
 * <pre>{@code
 * DifferentialMotorConstants<TalonFXConfiguration> constants =
 *     new DifferentialMotorConstants<TalonFXConfiguration>()
 *         .withCANBusName("canivore")
 *         .withLeaderId(0)
 *         .withFollowerId(1)
 *         .withSensorToDifferentialRatio(2.0)
 *         .withLeaderInitialConfigs(leaderConfig)
 *         .withFollowerInitialConfigs(followerConfig);
 *
 * DifferentialMechanismReal wrist = new DifferentialMechanismReal(
 *     "Wrist",
 *     new DiffMechCharacteristics(DifferenceAxis.ROLL, AverageAxis.PITCH, 3.0,
 *                                 Degrees.of(0), Degrees.of(90)),
 *     constants);
 * }</pre>
 */
public class DifferentialMechanismReal
        extends DifferentialMechanism<MotorIO, MotorIO, AbsoluteEncoderIO> {

    // -----------------------------------------------------------------------
    // CTRE differential mechanism
    // -----------------------------------------------------------------------

    /**
     * The CTRE DifferentialMechanism that manages both TalonFX motors. All control requests go
     * through this object; never send commands directly to the leader or follower TalonFX.
     */
    private final com.ctre.phoenix6.mechanisms.DifferentialMechanism<TalonFX> ctreMechanism;

    // -----------------------------------------------------------------------
    // Pre-allocated control requests (reused every loop to minimize GC)
    // -----------------------------------------------------------------------

    /**
     * Average-axis Motion Magic position request (slot configurable at call time). Motion Magic
     * provides smooth, profiled motion on the average axis.
     */
    private final MotionMagicTorqueCurrentFOC avgPositionRequest =
            new MotionMagicTorqueCurrentFOC(0);

    /**
     * Differential-axis unprofiledposition request. Note: Motion Magic is NOT supported on the
     * differential axis per CTRE's API.
     */
    private final PositionTorqueCurrentFOC diffPositionRequest = new PositionTorqueCurrentFOC(0);

    /** Average-axis velocity request for velocity control mode. */
    private final VelocityTorqueCurrentFOC avgVelocityRequest = new VelocityTorqueCurrentFOC(0);

    /** Average-axis voltage request for open-loop control. */
    private final VoltageOut avgVoltageRequest = new VoltageOut(0).withEnableFOC(true);

    /** Differential-axis voltage request for open-loop control. */
    private final VoltageOut diffVoltageRequest = new VoltageOut(0).withEnableFOC(true);

    // -----------------------------------------------------------------------
    // Status signals read from the leader/follower for logging
    // -----------------------------------------------------------------------

    // Leader signals
    private final StatusSignal<Angle> leaderPosition;
    private final StatusSignal<AngularVelocity> leaderVelocity;
    private final StatusSignal<Voltage> leaderSupplyVoltage;
    private final StatusSignal<Current> leaderSupplyCurrent;
    private final StatusSignal<Current> leaderTorqueCurrent;
    private final StatusSignal<Temperature> leaderTemperature;

    // Follower signals
    private final StatusSignal<Angle> followerPosition;
    private final StatusSignal<AngularVelocity> followerVelocity;
    private final StatusSignal<Voltage> followerSupplyVoltage;
    private final StatusSignal<Current> followerSupplyCurrent;
    private final StatusSignal<Current> followerTorqueCurrent;
    private final StatusSignal<Temperature> followerTemperature;

    // Disconnect detection with debouncing (avoids false positives on brief glitches)
    private final Debouncer leaderDebouncer = new Debouncer(0.5);
    private final Debouncer followerDebouncer = new Debouncer(0.5);
    private final Alert leaderDisconnected;
    private final Alert followerDisconnected;

    // -----------------------------------------------------------------------
    // ThreadLocal used to pass the CTRE mechanism through the super() call chain
    // -----------------------------------------------------------------------

    /**
     * Java requires that {@code super()} be the very first statement in a constructor. This
     * ThreadLocal lets us create the CTRE mechanism inside a static helper method (called as a
     * super-constructor argument), store it temporarily, and then retrieve it once the super call
     * is complete.
     */
    private static final ThreadLocal<com.ctre.phoenix6.mechanisms.DifferentialMechanism<TalonFX>>
            INIT_HOLDER = new ThreadLocal<>();

    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    /**
     * Constructs a real differential mechanism.
     *
     * @param name logical name used for logging and visualization
     * @param characteristics physical characteristics (axes, gearing, starting position)
     * @param constants CTRE motor constants (device IDs, CAN bus, initial configs, gear ratios)
     */
    public DifferentialMechanismReal(
            String name,
            DiffMechCharacteristics characteristics,
            DifferentialMotorConstants<TalonFXConfiguration> constants) {
        // We use a ThreadLocal to bridge the CTRE mechanism across the super() boundary.
        // initLeaderIO creates the CTRE mechanism, stores it, and returns a no-op MotorIO.
        // initFollowerIO retrieves the stored mechanism and returns a second no-op MotorIO.
        // The actual motor I/O is handled directly in this class using CTRE status signals.
        super(
                name,
                characteristics,
                initLeaderIO(constants),
                initFollowerIO(),
                Optional.empty(),
                name);
        this.ctreMechanism = INIT_HOLDER.get();
        INIT_HOLDER.remove();

        // Cache status signal references from the CTRE mechanism's motors
        TalonFX leader = ctreMechanism.getLeader();
        TalonFX follower = ctreMechanism.getFollower();

        leaderPosition = leader.getPosition();
        leaderVelocity = leader.getVelocity();
        leaderSupplyVoltage = leader.getSupplyVoltage();
        leaderSupplyCurrent = leader.getSupplyCurrent();
        leaderTorqueCurrent = leader.getTorqueCurrent();
        leaderTemperature = leader.getDeviceTemp();

        followerPosition = follower.getPosition();
        followerVelocity = follower.getVelocity();
        followerSupplyVoltage = follower.getSupplyVoltage();
        followerSupplyCurrent = follower.getSupplyCurrent();
        followerTorqueCurrent = follower.getTorqueCurrent();
        followerTemperature = follower.getDeviceTemp();

        leaderDisconnected = new Alert(name + " leader motor disconnected!", AlertType.kError);
        followerDisconnected = new Alert(name + " follower motor disconnected!", AlertType.kError);
    }

    // -----------------------------------------------------------------------
    // Static helpers for the constructor chain (ThreadLocal bridge)
    // -----------------------------------------------------------------------

    /**
     * Creates the CTRE DifferentialMechanism, stores it in the ThreadLocal, and returns a no-op
     * MotorIO placeholder for the super() call.
     */
    private static MotorIO initLeaderIO(
            DifferentialMotorConstants<TalonFXConfiguration> constants) {
        var mech =
                new com.ctre.phoenix6.mechanisms.DifferentialMechanism<TalonFX>(
                        TalonFX::new, constants);
        INIT_HOLDER.set(mech);
        return new MotorIO() {}; // no-op; actual input reading is done via CTRE status signals
    }

    /**
     * Returns a no-op MotorIO for the follower placeholder. The CTRE mechanism was already stored
     * in the ThreadLocal by {@link #initLeaderIO}.
     */
    private static MotorIO initFollowerIO() {
        return new MotorIO() {}; // no-op; same as leader
    }

    // -----------------------------------------------------------------------
    // Periodic — fault protection + logging
    // -----------------------------------------------------------------------

    @Override
    public void periodic() {
        // 1. Run CTRE's built-in fault detection FIRST. This monitors for dangerous conditions
        //    (e.g., motor disconnect, power cycle) and automatically disables the mechanism if
        //    needed. Always call this before sending new control requests.
        ctreMechanism.periodic();

        // 2. Read and log the leader motor inputs manually (bypassing the no-op MotorIO).
        boolean leaderConnected =
                BaseStatusSignal.refreshAll(
                                leaderPosition,
                                leaderVelocity,
                                leaderSupplyVoltage,
                                leaderSupplyCurrent,
                                leaderTorqueCurrent,
                                leaderTemperature)
                        .isOK();
        leaderDisconnected.set(leaderDebouncer.calculate(!leaderConnected));

        inputs.connected = leaderConnected;
        inputs.position = leaderPosition.getValue();
        inputs.velocity = leaderVelocity.getValue();
        inputs.appliedVoltage = leaderSupplyVoltage.getValue();
        inputs.supplyCurrent = leaderSupplyCurrent.getValue();
        inputs.torqueCurrent = leaderTorqueCurrent.getValue();
        inputs.temperature = leaderTemperature.getValue();
        Logger.processInputs(name, inputs);

        // 3. Read and log the follower motor inputs.
        boolean followerConnected =
                BaseStatusSignal.refreshAll(
                                followerPosition,
                                followerVelocity,
                                followerSupplyVoltage,
                                followerSupplyCurrent,
                                followerTorqueCurrent,
                                followerTemperature)
                        .isOK();
        followerDisconnected.set(followerDebouncer.calculate(!followerConnected));

        followerInputs.connected = followerConnected;
        followerInputs.position = followerPosition.getValue();
        followerInputs.velocity = followerVelocity.getValue();
        followerInputs.appliedVoltage = followerSupplyVoltage.getValue();
        followerInputs.supplyCurrent = followerSupplyCurrent.getValue();
        followerInputs.torqueCurrent = followerTorqueCurrent.getValue();
        followerInputs.temperature = followerTemperature.getValue();
        Logger.processInputs(name + "/Follower", followerInputs);

        // 4. Read and log the combined differential axis signals.
        updateDifferentialInputs(diffMechInputs);
        Logger.processInputs(name + "/DifferentialAxes", diffMechInputs);

        // 5. Update the 2D visualizer.
        visualizer.setAverageAngle(diffMechInputs.averagePosition);
        visualizer.setDifferentialAngle(diffMechInputs.differentialPosition);
    }

    // -----------------------------------------------------------------------
    // updateDifferentialInputs — reads CTRE mechanism's combined signals
    // -----------------------------------------------------------------------

    @Override
    protected void updateDifferentialInputs(DifferentialMechanismInputsAutoLogged inputs) {
        // Refresh the average/differential position and velocity from CTRE.
        // These are computed internally by CTRE from both motors' positions.
        inputs.averagePosition = ctreMechanism.getAveragePosition().getValue();
        inputs.averageVelocity = ctreMechanism.getAverageVelocity().getValue();
        inputs.differentialPosition = ctreMechanism.getDifferentialPosition().getValue();
        inputs.differentialVelocity = ctreMechanism.getDifferentialVelocity().getValue();

        // Closed-loop reference and error for both axes (useful for tuning)
        inputs.averageClosedLoopReference =
                ctreMechanism.getAverageClosedLoopReference().getValue();
        inputs.averageClosedLoopError = ctreMechanism.getAverageClosedLoopError().getValue();
        inputs.differentialClosedLoopReference =
                ctreMechanism.getDifferentialClosedLoopReference().getValue();
        inputs.differentialClosedLoopError =
                ctreMechanism.getDifferentialClosedLoopError().getValue();

        // Fault / safety state (populated by ctreMechanism.periodic())
        inputs.isDisabled = ctreMechanism.isDisabled();
        inputs.requiresUserAction = ctreMechanism.requiresUserAction();
    }

    // -----------------------------------------------------------------------
    // Differential control methods
    // -----------------------------------------------------------------------

    /**
     * {@inheritDoc}
     *
     * <p>Sends a paired open-loop voltage request: {@code averageVoltage} applied as the common
     * output to both motors and {@code differentialVoltage} as the opposing output.
     */
    @Override
    public void runVoltage(Voltage averageVoltage, Voltage differentialVoltage) {
        ctreMechanism.setControl(
                avgVoltageRequest.withOutput(averageVoltage.in(Volts)),
                diffVoltageRequest.withOutput(differentialVoltage.in(Volts)));
    }

    /**
     * {@inheritDoc}
     *
     * <p>Uses Motion Magic® on the average axis (smooth S-curve profiling) and a simple position
     * controller on the differential axis. The PID gains for each slot should be configured in the
     * {@link TalonFXConfiguration} passed to {@link DifferentialMotorConstants}.
     */
    @Override
    public void runPosition(
            Angle averagePosition,
            Angle differentialPosition,
            PIDSlot averageSlot,
            PIDSlot differentialSlot) {
        ctreMechanism.setControl(
                avgPositionRequest.withPosition(averagePosition).withSlot(averageSlot.getNum()),
                diffPositionRequest
                        .withPosition(differentialPosition)
                        .withSlot(differentialSlot.getNum()));
    }

    /**
     * {@inheritDoc}
     *
     * <p>Drives the average axis at the requested velocity while holding the differential axis at a
     * fixed position. Useful for shooting-while-moving or differential intake scenarios.
     */
    @Override
    public void runVelocityAverage(
            AngularVelocity averageVelocity,
            Angle differentialPosition,
            PIDSlot averageSlot,
            PIDSlot differentialSlot) {
        ctreMechanism.setControl(
                avgVelocityRequest.withVelocity(averageVelocity).withSlot(averageSlot.getNum()),
                diffPositionRequest
                        .withPosition(differentialPosition)
                        .withSlot(differentialSlot.getNum()));
    }

    /**
     * Stops both motors using the configured neutral mode (brake or coast). Delegates to CTRE's
     * {@code setNeutralOut()} so both motors stop simultaneously.
     */
    @Override
    public void runBrake() {
        ctreMechanism.setNeutralOut();
    }

    /**
     * Forces both motors to coast by issuing CTRE's {@code setCoastOut()} request, regardless of
     * the configured neutral mode.
     */
    @Override
    public void runCoast() {
        ctreMechanism.setCoastOut();
    }

    // -----------------------------------------------------------------------
    // Utility / configuration helpers
    // -----------------------------------------------------------------------

    /**
     * Resets the mechanism's sensor positions. Call this after physically recalibrating the
     * mechanism to a known position.
     *
     * @param averagePosition the new average-axis position
     * @param differentialPosition the new differential-axis position
     */
    public void setPosition(Angle averagePosition, Angle differentialPosition) {
        ctreMechanism.setPosition(averagePosition, differentialPosition);
    }

    /**
     * Changes the neutral mode for both motors after construction.
     *
     * @param neutralMode {@link NeutralModeValue#Brake} or {@link NeutralModeValue#Coast}
     */
    public void configNeutralMode(NeutralModeValue neutralMode) {
        ctreMechanism.configNeutralMode(neutralMode);
    }

    /**
     * Informs the CTRE mechanism that the user has performed the required safety action (e.g.,
     * called {@link #setPosition}) and that it is safe to re-enable control. Only call this when
     * you are certain the mechanism is in a known-good state.
     */
    public void clearUserRequirement() {
        ctreMechanism.clearUserRequirement();
    }

    /** Returns the raw CTRE mechanism, for advanced use cases not covered by this wrapper. */
    public com.ctre.phoenix6.mechanisms.DifferentialMechanism<TalonFX> getCtreMechanism() {
        return ctreMechanism;
    }
}
