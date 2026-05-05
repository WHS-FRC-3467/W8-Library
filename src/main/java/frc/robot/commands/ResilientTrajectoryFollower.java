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

package frc.robot.commands;

import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.lib.util.FieldUtil;
import frc.lib.util.LoggedTrigger;
import frc.lib.util.LoggedTunableBoolean;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;

import org.littletonrobotics.junction.Logger;

import java.util.Comparator;
import java.util.List;
import java.util.Map;

/**
 * A trajectory follower that pauses the trajectory timer when the robot deviates too far from the
 * path and resumes once it catches up. This replaces the old approach of abandoning the trajectory
 * and pathfinding back to the endpoint.
 *
 * <p>Two states:
 *
 * <ul>
 *   <li><b>TRACKING</b> — Normal following. Trajectory time advances with the wall clock.
 *       Feedforward velocities from the trajectory sample are applied alongside PID correction.
 *   <li><b>RECOVERING</b> — Trajectory time is frozen. The robot drives with PID-only toward the
 *       frozen sample pose (no feedforward, since the sample's velocities are stale). Once the
 *       robot is close enough, it transitions back to TRACKING.
 * </ul>
 *
 * <p>Hysteresis prevents oscillation: the pause threshold is larger than the resume threshold.
 * Events bound to trajectory timestamps naturally pause during recovery since the trajectory timer
 * is frozen.
 */
public class ResilientTrajectoryFollower extends Command {

    /** Current state of the follower. */
    public enum State {
        TRACKING,
        RECOVERING
    }

    private static final LoggedTunableNumber PAUSE_THRESHOLD_METERS =
            new LoggedTunableNumber("Auto/ResilientFollower/PauseThresholdMeters", 0.37);
    private static final LoggedTunableNumber RESUME_THRESHOLD_METERS =
            new LoggedTunableNumber("Auto/ResilientFollower/ResumeThresholdMeters", 0.15);
    private static final LoggedTunableNumber PAUSE_DEBOUNCE_SECONDS =
            new LoggedTunableNumber("Auto/ResilientFollower/PauseDebouncerSeconds", 0.25);

    /**
     * When true, the follower commands zero velocity to the drive, simulating the robot being stuck
     * or colliding with another robot. The trajectory timer still freezes via the normal RECOVERING
     * logic once the error threshold is exceeded. Toggle via NetworkTables for sim testing.
     */
    private static final LoggedTunableBoolean SIM_FORCE_STUCK =
            new LoggedTunableBoolean("Auto/ResilientFollower/SimForceStuck", false);

    /**
     * When toggled from false → true, the robot's pose is instantly displaced perpendicular to its
     * current heading by {@link #SIM_NUDGE_DISTANCE_METERS}, simulating a collision with another
     * robot. Toggle back to false and then true again to apply another nudge. This lets you test
     * the RECOVERING state without needing to hold the robot in place.
     */
    private static final LoggedTunableBoolean SIM_NUDGE_OFF_PATH =
            new LoggedTunableBoolean("Auto/ResilientFollower/SimNudgeOffPath", false);

    /**
     * The distance (in meters) that the robot is displaced when {@link #SIM_NUDGE_OFF_PATH} is
     * toggled. The displacement is applied perpendicular (to the left) of the robot's current
     * heading.
     */
    private static final LoggedTunableNumber SIM_NUDGE_DISTANCE_METERS =
            new LoggedTunableNumber("Auto/ResilientFollower/SimNudgeDistanceMeters", 0.5);

    private final Drive drive;
    private final RobotState robotState;
    private final Trajectory<SwerveSample> trajectory;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;

    /** Event bindings: event name → Command to schedule when the event fires. */
    private final Map<String, Command> eventBindings;

    /** Events sorted by timestamp for efficient sequential firing. */
    private final List<EventMarker> sortedEvents;

    private State state = State.TRACKING;

    /**
     * Set to {@code true} in {@link #end(boolean)} when the command finishes without interruption.
     * Reset to {@code false} in {@link #initialize()}. Polled by the {@link LoggedTrigger} returned
     * from {@link #done()}.
     */
    private boolean finished = false;

    /** Trajectory-local time (seconds). Advances only in TRACKING state. */
    private double trajectoryTime;

    /** Wall-clock timestamp from the previous execute() call, for computing dt. */
    private double previousTimestamp;

    /** Cumulative time spent in RECOVERING state (for diagnostics). */
    private double totalPausedTime;

    /** Index of the next event to fire (into {@link #sortedEvents}). */
    private int nextEventIndex;

    /** Debouncer for the TRACKING → RECOVERING transition. */
    private Debouncer pauseDebouncer;

    /** Tracks the previous state of {@link #SIM_NUDGE_OFF_PATH} to detect rising edges. */
    private boolean previousNudgeState;

    /**
     * Creates a new ResilientTrajectoryFollower.
     *
     * @param drive the drive subsystem (required by this command)
     * @param trajectory the Choreo trajectory to follow
     * @param xController PID controller for X translation
     * @param yController PID controller for Y translation
     * @param thetaController PID controller for rotation
     * @param eventBindings map of event name to Command to schedule at that event's timestamp
     */
    public ResilientTrajectoryFollower(
            Drive drive,
            Trajectory<SwerveSample> trajectory,
            PIDController xController,
            PIDController yController,
            PIDController thetaController,
            Map<String, Command> eventBindings) {
        this.drive = drive;
        this.robotState = RobotState.getInstance();
        this.trajectory = trajectory;
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;
        this.eventBindings = eventBindings;

        // Pre-sort events by timestamp for sequential firing.
        this.sortedEvents =
                trajectory.events().stream()
                        .sorted(Comparator.comparingDouble(e -> e.timestamp))
                        .toList();

        addRequirements(drive);
        setName("ResilientFollow_" + trajectory.name());
    }

    /**
     * Returns a {@link LoggedTrigger} that becomes {@code true} once this command finishes
     * successfully (i.e. not interrupted). The trigger resets to {@code false} when the command is
     * re-initialized.
     *
     * <p>Use this in auto routines to chain phases together:
     *
     * <pre>{@code
     * ResilientTrajectoryFollower follow = drive.followTrajectoryResilient(traj, events);
     * follow.done().onTrue(nextPhaseCommand);
     * }</pre>
     *
     * @return a trigger that is active when this command has completed successfully
     */
    public LoggedTrigger done() {
        return new LoggedTrigger(
                "Auto/ResilientFollow_" + trajectory.name() + "/Done", () -> finished);
    }

    @Override
    public void initialize() {
        double time = Timer.getFPGATimestamp();
        finished = false;
        state = State.TRACKING;
        trajectoryTime = 0.0;
        previousTimestamp = Timer.getTimestamp();
        totalPausedTime = 0.0;
        nextEventIndex = 0;
        pauseDebouncer = new Debouncer(PAUSE_DEBOUNCE_SECONDS.get());
        previousNudgeState = SIM_NUDGE_OFF_PATH.get();

        xController.reset();
        yController.reset();
        thetaController.reset();

        Logger.recordOutput("Odometry/Trajectory", FieldUtil.apply(trajectory.getPoses()));
        Logger.recordOutput("Auto/ResilientFollower/State", state.name());
        Logger.recordOutput("Auto/Init Delay", Timer.getFPGATimestamp() - time);
    }

    @Override
    public void execute() {
        double now = Timer.getTimestamp();
        double dt = now - previousTimestamp;
        previousTimestamp = now;

        Pose2d currentPose = robotState.getEstimatedPose();

        // --- Sim nudge: displace the robot perpendicular to its heading on rising edge ---
        // Guard with isSimulation() so a stray NT write on a real robot can't jump odometry.
        boolean nudgeNow = RobotBase.isSimulation() && SIM_NUDGE_OFF_PATH.get();
        if (nudgeNow && !previousNudgeState) {
            double nudgeDistance = SIM_NUDGE_DISTANCE_METERS.get();
            // Offset 90° to the left of the robot's current heading.
            Rotation2d perpendicular = currentPose.getRotation().plus(Rotation2d.kCCW_90deg);
            Translation2d nudgedTranslation =
                    currentPose
                            .getTranslation()
                            .plus(new Translation2d(nudgeDistance, perpendicular));
            Pose2d nudgedPose = new Pose2d(nudgedTranslation, currentPose.getRotation());
            robotState.resetPose(nudgedPose);
            currentPose = nudgedPose;
            Logger.recordOutput("Auto/ResilientFollower/SimNudgeApplied", true);
        } else {
            Logger.recordOutput("Auto/ResilientFollower/SimNudgeApplied", false);
        }
        previousNudgeState = nudgeNow;

        // Sample the trajectory at the current trajectory time, flipping for red alliance.
        boolean flip = FieldUtil.shouldFlip();
        SwerveSample sample =
                trajectory
                        .sampleAt(trajectoryTime, flip)
                        .orElseGet(() -> trajectory.getFinalSample(flip).orElse(null));

        if (sample == null) {
            drive.stop();
            return;
        }

        Pose2d targetPose = sample.getPose();
        double preAdvanceTranslationalError =
                currentPose.getTranslation().getDistance(targetPose.getTranslation());

        // --- State transitions ---
        State previousState = state;
        updateState(preAdvanceTranslationalError);

        // --- Advance trajectory time only if we were TRACKING at the start of this cycle ---
        // This ensures that the dt spent in RECOVERING (including the transition cycle) is
        // not added to trajectoryTime, keeping trajectory time fully frozen during recovery.
        if (previousState == State.TRACKING) {
            trajectoryTime += dt;
            // Clamp to trajectory end so we don't overshoot into invalid territory.
            trajectoryTime = Math.min(trajectoryTime, trajectory.getTotalTime());

            // Re-sample after advancing time so we're using the latest target.
            sample =
                    trajectory
                            .sampleAt(trajectoryTime, flip)
                            .orElseGet(() -> trajectory.getFinalSample(flip).orElse(null));
            if (sample == null) {
                drive.stop();
                return;
            }
            targetPose = sample.getPose();
        } else {
            totalPausedTime += dt;
        }

        // Recompute error against the final target pose used this cycle for accurate logging.
        double translationalError =
                currentPose.getTranslation().getDistance(targetPose.getTranslation());

        // --- Fire events ---
        fireEvents();

        // --- Compute and apply chassis speeds ---
        ChassisSpeeds targetSpeeds;
        if (state == State.TRACKING) {
            // Normal following: feedforward + PID feedback
            targetSpeeds =
                    new ChassisSpeeds(
                            sample.vx + xController.calculate(currentPose.getX(), sample.x),
                            sample.vy + yController.calculate(currentPose.getY(), sample.y),
                            sample.omega
                                    + thetaController.calculate(
                                            currentPose.getRotation().getRadians(),
                                            sample.heading));
        } else {
            // Recovery: PID-only toward frozen target pose (no feedforward)
            targetSpeeds =
                    new ChassisSpeeds(
                            xController.calculate(currentPose.getX(), sample.x),
                            yController.calculate(currentPose.getY(), sample.y),
                            thetaController.calculate(
                                    currentPose.getRotation().getRadians(), sample.heading));
        }

        // When SimForceStuck is enabled, command zero velocity to mimic being pinned.
        if (RobotBase.isSimulation() && SIM_FORCE_STUCK.get()) {
            drive.stop();
        } else {
            drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, currentPose.getRotation()));
        }
        robotState.setActiveTrajPose(targetPose);

        // --- Logging ---
        Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        Logger.recordOutput("Auto/ResilientFollower/State", state.name());
        Logger.recordOutput("Auto/ResilientFollower/TrajectoryTime", trajectoryTime);
        Logger.recordOutput("Auto/ResilientFollower/TotalPausedTime", totalPausedTime);
        Logger.recordOutput("Auto/ResilientFollower/TranslationalError", translationalError);
    }

    private void updateState(double translationalError) {
        switch (state) {
            case TRACKING -> {
                boolean overThreshold =
                        pauseDebouncer.calculate(translationalError > PAUSE_THRESHOLD_METERS.get());
                if (overThreshold) {
                    state = State.RECOVERING;
                    // Reset PID integrators to avoid windup from accumulated error.
                    xController.reset();
                    yController.reset();
                    thetaController.reset();
                }
            }
            case RECOVERING -> {
                if (translationalError < RESUME_THRESHOLD_METERS.get()) {
                    state = State.TRACKING;
                    // Reset debouncer so momentary spikes after resuming don't re-trigger.
                    pauseDebouncer = new Debouncer(PAUSE_DEBOUNCE_SECONDS.get());
                    // Reset PID integrators for clean tracking restart.
                    xController.reset();
                    yController.reset();
                    thetaController.reset();
                }
            }
        }
    }

    /** Fires any events whose timestamp has been reached by trajectoryTime. */
    private void fireEvents() {
        while (nextEventIndex < sortedEvents.size()) {
            EventMarker event = sortedEvents.get(nextEventIndex);
            if (trajectoryTime >= event.timestamp) {
                Command bound = eventBindings.get(event.event);
                if (bound != null) {
                    CommandScheduler.getInstance().schedule(bound);
                }
                nextEventIndex++;
            } else {
                break;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        finished = !interrupted;
        drive.stop();
        Logger.recordOutput("Odometry/Trajectory", new Pose2d[] {});
        Logger.recordOutput("Odometry/TrajectorySetpoint", new Pose2d());
        Logger.recordOutput("Auto/ResilientFollower/State", "DONE");
    }

    @Override
    public boolean isFinished() {
        // Finished when trajectory time has passed the end AND we are close enough to the final
        // pose (either in TRACKING state or within resume threshold).
        if (trajectoryTime < trajectory.getTotalTime()) {
            return false;
        }
        Pose2d currentPose = robotState.getEstimatedPose();
        Pose2d finalPose = trajectory.getFinalPose(FieldUtil.shouldFlip()).orElse(new Pose2d());
        double error = currentPose.getTranslation().getDistance(finalPose.getTranslation());
        return error < RESUME_THRESHOLD_METERS.get();
    }
}
