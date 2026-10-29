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

package frc.lib.posestimator;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;

import frc.lib.posestimator.PoseEstimator.VisionPoseObservation;
import frc.lib.posestimator.SwerveOdometry.OdometryObservation;

import lombok.Builder;

import org.ejml.data.SingularMatrixException;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Deque;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;
import java.util.function.Predicate;

/**
 * Extended Kalman filter (EKF) pose estimator for a swerve drive. Drop-in alternative to {@link
 * PoseEstimator} that exposes the same public API and consumes the same {@link
 * VisionPoseObservation}.
 *
 * <h2>Model</h2>
 *
 * <ul>
 *   <li><b>Pose</b> {@code [x, y, heading]}, field-relative.
 *   <li><b>Uncertainty</b>: a 3x3 covariance table. The diagonal is the variance of x, y and
 *       heading; the off-diagonal entries are how errors in different axes move together.
 *   <li><b>Odometry twist</b> {@code [dx, dy, dheading]}: the robot-frame motion measured by wheel
 *       odometry and the gyro over one sample.
 *   <li><b>Motion model</b> {@code pose.exp(twist)}, which is nonlinear in heading.
 *   <li><b>Vision measurement</b>: a field-relative pose, so every axis is observed directly.
 * </ul>
 *
 * <h2>Predict (every odometry sample)</h2>
 *
 * <pre>
 * pose'        = pose.exp(odometryTwist)
 * uncertainty' = errorCarryOver · uncertainty · errorCarryOverᵀ
 *              + noiseCarryOver · odometryNoise · noiseCarryOverᵀ
 *
 * errorCarryOver = [1 0 −Δy_field]   Δ_field is the field-frame displacement of this step, so
 *                  [0 1  Δx_field]   heading uncertainty turns into position uncertainty in
 *                  [0 0  1       ]   proportion to how far the robot just moved.
 *
 * noiseCarryOver = [rotation(heading) · V(dheading)  0]   Rotates robot-frame odometry noise into
 *                  [0                                1]   the field frame. V is the SE(2) left
 *                                                         Jacobian that {@code Pose2d.exp} applies.
 * </pre>
 *
 * <p>The odometry noise is built from {@link Config} so that uncertainty grows with distance
 * travelled and with time.
 *
 * <h2>Update (every vision observation)</h2>
 *
 * <pre>
 * gap         = cameraPose − pose                    (heading wrapped to ±π)
 * expectedGap = uncertainty + cameraUncertainty      (measured axes only)
 * gain        = uncertainty · expectedGap⁻¹
 * pose        = pose + gain · gap
 * uncertainty = (I − gain) · uncertainty · (I − gain)ᵀ
 *             + gain · cameraUncertainty · gainᵀ     (Joseph form, stays symmetric positive definite)
 * </pre>
 *
 * <p>The camera uncertainty is built from the standard deviations carried by each {@link
 * VisionPoseObservation}, scaled by {@link Config#visionStdDevScale()}.
 *
 * <h2>Latency compensation</h2>
 *
 * <p>Each odometry sample stores the state and covariance after that sample. A vision observation
 * is applied at the latest sample at or before its capture timestamp (so the covariance used is the
 * one the filter had at that moment), then every later odometry step is replayed through the
 * predict equations. The timing error is at most one odometry period.
 *
 * <h2>Outlier gating</h2>
 *
 * <p>A vision frame's surprise is its NIS (normalized innovation squared): {@code gapᵀ ·
 * expectedGap⁻¹ · gap}, the gap between the camera and the estimate normalized by the gap expected
 * from the filter's uncertainty plus the camera's. When those uncertainties are honest, it is
 * chi-squared distributed with one degree of freedom per measured axis (3 for x, y and heading), so
 * a threshold has a known false-rejection rate. Frames above {@link Config#gateChiSquared()} are
 * rejected. A filter that is too sure of itself will reject the very frames that would correct it,
 * so the gate also watches for persistent disagreement: when at least {@link
 * Config#gateRecoveryCount()} observations, and at least {@link Config#gateRecoveryFraction()} of
 * the observations, within the last {@link Config#gateRecoveryWindowSeconds()} were rejected, the
 * filter assumes it is the one that is wrong (for example the robot slipped or was shoved). It then
 * inflates its covariance by the smallest factor that lets the current observation through, and
 * accepts it. The window is a fraction of recent observations and not a run of consecutive
 * rejections, so a few accepted low-quality observations mixed in cannot hold the filter in the
 * wrong place.
 *
 * <p>See {@code docs/EkfPoseEstimator.md} for the tuning procedure.
 */
public class EkfPoseEstimator {
    private static final String LOG_PREFIX = "EkfPoseEstimator/";

    private static final double DEFAULT_ODOMETRY_BUFFER_SIZE_SECONDS = 2.0;

    /**
     * Width of the window {@link #getVelocityAtTime} averages over, centered on the requested time.
     * One odometry step is a few milliseconds, which is too noisy to read a velocity from.
     */
    private static final double VELOCITY_WINDOW_SECONDS = 0.05;

    /** Below this heading change (rad) the arc terms use their Taylor expansion. */
    private static final double ARC_EPSILON = 1e-9;

    /**
     * All tunable parameters of the filter. Immutable, build a modified copy with {@code
     * Config.DEFAULT.toBuilder().linearVariancePerMeter(0.004).build()} and apply it with {@link
     * EkfPoseEstimator#setConfig(Config)}.
     *
     * <p>Variances are the square of the corresponding standard deviation. Variance, not standard
     * deviation, is what adds up over time, which is why the process noise parameters are rates of
     * variance.
     *
     * @param linearVariancePerMeter Process noise: translation variance added per meter driven, in
     *     m²/m. Models wheel slip, scale error and scrub. After driving {@code d} meters the
     *     translation standard deviation has grown by about {@code sqrt(linearVariancePerMeter *
     *     d)}. <b>Raise</b> if the estimate lags behind vision while driving, or if high-quality
     *     vision frames are being gated (the filter is overconfident in odometry); <b>lower</b> if
     *     the pose jitters when vision updates arrive. A higher value fixes slip bursts but pulls
     *     the pose toward vision frames that are about 0.2 s old while the robot is fast; the
     *     default is the knee of that tradeoff on one logged match (see docs/EkfPoseEstimator.md).
     * @param linearVariancePerSecond Process noise: translation variance added per second,
     *     regardless of motion, in m²/s. A small floor that stops the uncertainty from collapsing
     *     to zero after many vision updates, so the filter never becomes completely deaf to vision.
     *     Keep it small.
     * @param angularVariancePerSecond Process noise: heading variance added per second, in rad²/s.
     *     Models gyro drift. This is the main knob for how much vision is allowed to correct
     *     heading. A good gyro wants a small value, which makes the filter mostly ignore vision
     *     heading. <b>Raise</b> if heading drifts and vision cannot pull it back.
     * @param initialLinearStdDev Initial position uncertainty at boot, in meters. The robot does
     *     not know where it is until vision or a reset says so, so this should be large.
     * @param initialAngularStdDev Initial heading uncertainty at boot, in radians.
     * @param resetLinearStdDev Position uncertainty after {@link
     *     EkfPoseEstimator#resetPose(Pose2d)}, in meters. Match how accurately the reset pose is
     *     actually known (an auto starting position placed by hand is good to a few centimeters).
     * @param resetAngularStdDev Heading uncertainty after a reset, in radians.
     * @param visionStdDevScale Multiplier applied to every vision standard deviation (both linear
     *     and angular). A single global trust knob for vision. Values above 1 trust vision less.
     * @param gateChiSquared Largest NIS (see the class documentation) a vision frame may have
     *     before it is rejected. The NIS is unitless: the gap between camera and estimate,
     *     normalized by the gap expected from the filter's uncertainty plus the camera's, so the
     *     same 0.3 m gap is routine when the filter is unsure and alarming when it is sure.
     *     <p>With honest uncertainties the NIS is chi-squared distributed with one degree of
     *     freedom per measured axis, so a full pose (x, y, heading) has 3 and a typical NIS of
     *     about 3. This is the cutoff on that distribution, written for 3 degrees of freedom. The
     *     fraction of <i>good</i> frames that exceed it:
     *     <ul>
     *       <li>7.81: 5%
     *       <li>11.34: 1%
     *       <li>16.27: 0.1% (the default)
     *       <li>21.11: 0.01%
     *     </ul>
     *     <p>A frame that measures fewer axes has fewer degrees of freedom (a single tag with its
     *     heading ignored measures x and y: 2), so the cutoff is recomputed to reject the same
     *     fraction of good frames: 16.27 becomes 13.82 for 2 axes and 10.83 for 1.
     *     <p>A filter that is too sure of itself rejects exactly the frames that would correct it.
     *     If good frames are being rejected, raise {@code linearVariancePerMeter} instead of this.
     *     With a consistent filter, {@code Vision/Nis} averages about 3 for full-pose frames and
     *     about 0.1% of frames are rejected. Sustained rejection is handled by the {@code
     *     gateRecovery*} settings.
     *     <p>Use {@code Double.POSITIVE_INFINITY} to disable gating.
     * @param gateRecoveryCount Minimum number of gated observations inside the recovery window
     *     before the filter may conclude that it, not vision, is wrong. Must be at least 1. A
     *     larger value is more resistant to a burst of bad frames but is slower to recover after
     *     the robot slips or is moved by something the model cannot see.
     * @param gateRecoveryWindowSeconds Length of the recovery window, in seconds of observation
     *     capture time. Must be positive. Rejections older than this stop counting.
     * @param gateRecoveryFraction Fraction of the observations inside the recovery window that must
     *     have been gated, in (0, 1]. Keeps a camera that is occasionally wrong from triggering
     *     recovery while good frames are still being accepted.
     * @param tippedLinearVariancePerSecond Translation variance added per second while tipped, in
     *     m²/s. Wheels slip or leave the floor, so odometry is not to be believed, and this grows
     *     the filter's uncertainty in proportion to how long the robot was tipped. That is what
     *     makes the frames during and after a tip count for much more: the longer the tip, the
     *     harder they pull. Vision frames are not otherwise treated differently while tipped.
     */
    @Builder(toBuilder = true)
    public record Config(
            double linearVariancePerMeter,
            double linearVariancePerSecond,
            double angularVariancePerSecond,
            double initialLinearStdDev,
            double initialAngularStdDev,
            double resetLinearStdDev,
            double resetAngularStdDev,
            double visionStdDevScale,
            double gateChiSquared,
            int gateRecoveryCount,
            double gateRecoveryWindowSeconds,
            double gateRecoveryFraction,
            double tippedLinearVariancePerSecond) {

        /** Starting values. Expect to retune them on the real robot. */
        public static final Config DEFAULT =
                Config.builder()
                        // sigma = 14 cm after 1 m and 45 cm after 10 m of unaided odometry
                        .linearVariancePerMeter(0.02)
                        // sigma = 2.4 cm after 60 s standing still
                        .linearVariancePerSecond(1e-5)
                        // sigma = 0.6 degrees after 10 s
                        .angularVariancePerSecond(1e-5)
                        .initialLinearStdDev(1.0)
                        .initialAngularStdDev(0.5)
                        .resetLinearStdDev(0.05)
                        // about 1 degree
                        .resetAngularStdDev(0.02)
                        .visionStdDevScale(1.0)
                        // 99.9% for 3 DOF
                        .gateChiSquared(16.27)
                        .gateRecoveryCount(5)
                        .gateRecoveryWindowSeconds(1.5)
                        .gateRecoveryFraction(0.5)
                        .tippedLinearVariancePerSecond(0.5)
                        .build();

        public Config {
            requireNonNegative("linearVariancePerMeter", linearVariancePerMeter);
            requireNonNegative("linearVariancePerSecond", linearVariancePerSecond);
            requireNonNegative("angularVariancePerSecond", angularVariancePerSecond);
            requireNonNegative("initialLinearStdDev", initialLinearStdDev);
            requireNonNegative("initialAngularStdDev", initialAngularStdDev);
            requireNonNegative("resetLinearStdDev", resetLinearStdDev);
            requireNonNegative("resetAngularStdDev", resetAngularStdDev);
            requireNonNegative("visionStdDevScale", visionStdDevScale);
            requireNonNegative("gateChiSquared", gateChiSquared);
            if (gateRecoveryCount < 1) {
                throw new IllegalArgumentException("gateRecoveryCount must be at least 1");
            }
            if (!(gateRecoveryWindowSeconds > 0.0)) {
                throw new IllegalArgumentException(
                        "gateRecoveryWindowSeconds must be positive, got "
                                + gateRecoveryWindowSeconds);
            }
            requireNonNegative("tippedLinearVariancePerSecond", tippedLinearVariancePerSecond);
            if (!(gateRecoveryFraction > 0.0 && gateRecoveryFraction <= 1.0)) {
                throw new IllegalArgumentException(
                        "gateRecoveryFraction must be in (0, 1], got " + gateRecoveryFraction);
            }
        }

        private static void requireNonNegative(String name, double value) {
            if (!(value >= 0.0)) {
                throw new IllegalArgumentException(name + " must be non-negative, got " + value);
            }
        }
    }

    enum VisionObservationReason {
        ACCEPTED,
        ACCEPTED_AFTER_GATE_RECOVERY,
        MISSING_ODOMETRY_SAMPLE,
        INVALID_OBSERVATION,
        NO_USABLE_MEASUREMENT,
        SINGULAR_INNOVATION_COVARIANCE,
        GATED_OUTLIER,
        POSE_VALIDATOR_REJECTED
    }

    /** Whether one vision observation was gated, kept to judge persistent disagreement. */
    private record GateOutcome(double timestampSeconds, boolean gated) {}

    /** Estimated pose and the covariance of that estimate. */
    private record State(Pose2d pose, Matrix<N3, N3> covariance) {}

    /**
     * One odometry sample: the twist and elapsed time that produced it, and the filter state
     * immediately after it, and whether the robot was tipped. The twist is kept so the step can be
     * replayed after a late vision correction, and the tipped flag so the replay adds the same
     * extra uncertainty.
     */
    private record Step(
            double timestampSeconds,
            Twist2d twist,
            double dtSeconds,
            boolean tipped,
            State state,
            List<Correction> corrections) {}

    /**
     * An accepted vision measurement, kept on the step it was applied at so that a frame arriving
     * late, and applied at an earlier step, can replay it over the new state instead of losing it.
     *
     * @param recovered whether gate recovery inflated the covariance to admit it, which the replay
     *     repeats against the new state
     */
    private record Correction(
            Pose2d measurement, double[] variances, boolean[] measured, boolean recovered) {}

    private final SwerveOdometry odometry;
    private final double bufferSeconds;
    private final Deque<Step> history = new ArrayDeque<>();

    private Config config;
    private double lastTimestampSeconds = Double.NaN;
    private boolean tipped = false;
    private final ArrayDeque<GateOutcome> recentGateOutcomes = new ArrayDeque<>();
    private Predicate<Pose2d> poseValidator = pose -> true;

    // gateThreshold bisects with erf and exp, so it is computed once per (dof, gateChiSquared)
    private final double[] cachedGateThresholds = new double[4];
    private double cachedGateChiSquared = Double.NaN;

    /** Outcome of the most recent vision observation, for tests and diagnostics. */
    VisionObservationReason lastVisionReason;

    /** NIS of the most recent vision observation, NaN if it was not computed. */
    double lastNis = Double.NaN;

    public EkfPoseEstimator(SwerveDriveKinematics kinematics) {
        this(kinematics, Config.DEFAULT);
    }

    public EkfPoseEstimator(SwerveDriveKinematics kinematics, Config config) {
        this(kinematics, Seconds.of(DEFAULT_ODOMETRY_BUFFER_SIZE_SECONDS), config);
    }

    /**
     * @param kinematics swerve kinematics used to turn module deltas into a twist
     * @param bufferSize how much odometry history to keep for latency compensation. Must exceed the
     *     worst-case camera latency.
     * @param config filter parameters
     */
    public EkfPoseEstimator(SwerveDriveKinematics kinematics, Time bufferSize, Config config) {
        this.config = config;
        this.bufferSeconds = bufferSize.in(Seconds);
        this.odometry = new SwerveOdometry(kinematics, bufferSize);

        // Initialize to a value not outside the field, same as PoseEstimator
        seedHistory(
                new Pose2d(1, 1, Rotation2d.kZero),
                config.initialLinearStdDev(),
                config.initialAngularStdDev());
    }

    // ---------------------------------------------------------------------------------------
    // Public API (mirrors PoseEstimator)
    // ---------------------------------------------------------------------------------------

    /** Current best estimate of the robot pose. */
    public Pose2d estimatedPose() {
        return latest().state().pose();
    }

    /** Odometry-only pose, with no vision corrections. */
    public Pose2d odometryPose() {
        return odometry.odometryPose();
    }

    /** Current 3x3 covariance of the estimate, ordered {@code [x, y, θ]}. */
    public Matrix<N3, N3> covariance() {
        return latest().state().covariance();
    }

    /**
     * Tells the filter whether the robot is tipped. It applies to odometry samples added after this
     * call. While tipped, odometry is trusted less ({@link
     * Config#tippedLinearVariancePerSecond()}), so vision pulls the pose harder during the tip and
     * in the first frames after it. Call it every cycle, before that cycle's odometry.
     */
    public void setTipped(boolean tipped) {
        this.tipped = tipped;
    }

    /** Current filter parameters. */
    public Config config() {
        return config;
    }

    /**
     * Replaces the filter parameters. Takes effect from the next odometry sample. Safe to call
     * every loop, for example from {@code LoggedTunableNumber.ifChanged}.
     */
    public void setConfig(Config config) {
        this.config = config;
    }

    /**
     * Adds a wheel odometry sample and runs the EKF predict step.
     *
     * @param observation module positions, optional gyro angle and timestamp
     */
    public void addOdometryObservation(OdometryObservation observation) {
        Pose2d lastOdometryPose = odometry.odometryPose();
        odometry.addOdometryObservation(observation);
        predictFromOdometry(lastOdometryPose, observation.timestamp().in(Seconds));
    }

    /**
     * Adds a gyro-only sample (translation is ignored) and runs the EKF predict step.
     *
     * @param observation optional gyro angle and timestamp
     */
    public void addGyroObservation(OdometryObservation observation) {
        Pose2d lastOdometryPose = odometry.odometryPose();
        odometry.addGyroObservation(observation);
        predictFromOdometry(lastOdometryPose, observation.timestamp().in(Seconds));
    }

    /**
     * Estimated pose at a past time, interpolated between odometry samples.
     *
     * @param timestampSeconds the time to look up, in the same timebase as odometry samples
     * @return the pose, or empty if that time is older than the retained history
     */
    public Optional<Pose2d> getPoseAtTime(double timestampSeconds) {
        Step before = null;
        Step after = null;
        Step newer = null;
        for (Iterator<Step> it = history.descendingIterator(); it.hasNext(); ) {
            Step step = it.next();
            if (step.timestampSeconds() <= timestampSeconds) {
                before = step;
                after = newer;
                break;
            }
            newer = step;
        }
        if (before == null) {
            return Optional.empty();
        }
        if (after == null) {
            return Optional.of(before.state().pose());
        }
        double span = after.timestampSeconds() - before.timestampSeconds();
        if (span <= 0.0 || Double.isInfinite(before.timestampSeconds())) {
            return Optional.of(before.state().pose());
        }
        double fraction = (timestampSeconds - before.timestampSeconds()) / span;
        return Optional.of(before.state().pose().interpolate(after.state().pose(), fraction));
    }

    /**
     * The robot's velocity around a past time, from the odometry steps near it, so a camera frame
     * can be judged by how fast the robot was moving when it was captured and not when it arrived.
     *
     * <p>The velocity is robot-relative, averaged over the steps within {@value
     * #VELOCITY_WINDOW_SECONDS} s centered on {@code timestampSeconds}. If no step falls in that
     * window, the nearest earlier step is used.
     *
     * @param timestampSeconds the time to look up, in the same timebase as odometry samples
     * @return the velocity, or empty if the retained history does not reach that time
     */
    public Optional<ChassisSpeeds> getVelocityAtTime(double timestampSeconds) {
        double from = timestampSeconds - VELOCITY_WINDOW_SECONDS / 2.0;
        double to = timestampSeconds + VELOCITY_WINDOW_SECONDS / 2.0;

        double dx = 0.0;
        double dy = 0.0;
        double dtheta = 0.0;
        double seconds = 0.0;
        // Nearest step at or before the time that covers some time, used if the window is empty
        Step fallback = null;
        boolean reached = false;
        // Newest to oldest, stopping once past the window, so only the recent steps are visited
        for (Iterator<Step> it = history.descendingIterator(); it.hasNext(); ) {
            Step step = it.next();
            double stamp = step.timestampSeconds();
            if (stamp <= timestampSeconds) {
                reached = true;
                if (fallback == null && step.dtSeconds() > 0.0) {
                    fallback = step;
                }
            }
            if (stamp >= from && stamp <= to && step.dtSeconds() > 0.0) {
                dx += step.twist().dx;
                dy += step.twist().dy;
                dtheta += step.twist().dtheta;
                seconds += step.dtSeconds();
            }
            if (stamp < from && fallback != null) {
                break;
            }
        }
        if (!reached) {
            return Optional.empty();
        }
        if (seconds <= 0.0) {
            if (fallback == null) {
                return Optional.empty();
            }
            dx = fallback.twist().dx;
            dy = fallback.twist().dy;
            dtheta = fallback.twist().dtheta;
            seconds = fallback.dtSeconds();
        }
        return Optional.of(new ChassisSpeeds(dx / seconds, dy / seconds, dtheta / seconds));
    }

    /**
     * Fuses a vision pose measurement taken at {@code observation.timestampSeconds()}.
     *
     * @param observation the vision observation. Its standard deviations define the camera
     *     uncertainty.
     */
    public void addVisionObservation(VisionPoseObservation observation) {
        int stepsAfter = stepsAfter(observation.timestampSeconds());
        if (stepsAfter < 0) {
            logVision(observation, VisionObservationReason.MISSING_ODOMETRY_SAMPLE, Double.NaN, 0);
            return;
        }

        Pose2d measurement = observation.robotPose();
        double[] values = {
            measurement.getX(), measurement.getY(), measurement.getRotation().getRadians()
        };
        double linearStdDev = observation.linearStdDev() * config.visionStdDevScale();
        double angularStdDev = observation.angularStdDev() * config.visionStdDevScale();
        if (!allFinite(values) || Double.isNaN(linearStdDev) || Double.isNaN(angularStdDev)) {
            logVision(observation, VisionObservationReason.INVALID_OBSERVATION, Double.NaN, 0);
            return;
        }

        // An infinite std dev means "no information about this axis" (VisionSubsystem uses it to
        // ignore heading from single-tag solves). That axis is left out of the update rather than
        // put into S, where an infinite variance cannot be inverted.
        double[] variances = {square(linearStdDev), square(linearStdDev), square(angularStdDev)};
        boolean[] measured = new boolean[3];
        int measuredAxes = 0;
        for (int i = 0; i < 3; i++) {
            measured[i] = Double.isFinite(variances[i]);
            if (measured[i]) {
                measuredAxes++;
            }
        }
        if (measuredAxes == 0) {
            logVision(observation, VisionObservationReason.NO_USABLE_MEASUREMENT, Double.NaN, 0);
            return;
        }

        // The anchor is the latest step at or before the capture time; `newer` is everything after
        Iterator<Step> back = history.descendingIterator();
        Step[] newer = new Step[stepsAfter];
        for (int i = stepsAfter - 1; i >= 0; i--) {
            newer[i] = back.next();
        }
        Step anchor = back.next();
        State prior = anchor.state();

        Matrix<N3, N3> observationMatrix = observationMatrix(measured);
        Matrix<N3, N3> measurementNoise = measurementNoise(variances, measured);
        Vector<N3> innovation = innovation(prior.pose(), measurement, measured);

        Matrix<N3, N3> innovationInverse;
        double nis;
        try {
            innovationInverse =
                    innovationCovarianceInverse(
                            prior.covariance(), observationMatrix, measurementNoise);
            nis = quadraticForm(innovation, innovationInverse);
        } catch (SingularMatrixException e) {
            logVision(
                    observation,
                    VisionObservationReason.SINGULAR_INNOVATION_COVARIANCE,
                    Double.NaN,
                    measuredAxes);
            return;
        }

        double threshold = gateThresholdFor(measuredAxes);
        boolean gated = nis > threshold;
        recordGateOutcome(observation.timestampSeconds(), gated);

        VisionObservationReason reason = VisionObservationReason.ACCEPTED;
        if (gated) {
            if (!shouldRecoverFromGating()) {
                logVision(observation, VisionObservationReason.GATED_OUTLIER, nis, measuredAxes);
                return;
            }
            // Vision has disagreed with the filter for a sustained stretch, so assume the filter is
            // the one that is wrong. Admit this observation with the least added uncertainty.
            try {
                Matrix<N3, N3> inflated =
                        inflateToGate(
                                prior.covariance(),
                                innovation,
                                observationMatrix,
                                measurementNoise,
                                threshold);
                innovationInverse =
                        innovationCovarianceInverse(inflated, observationMatrix, measurementNoise);
                prior = new State(prior.pose(), inflated);
            } catch (SingularMatrixException e) {
                logVision(
                        observation,
                        VisionObservationReason.SINGULAR_INNOVATION_COVARIANCE,
                        nis,
                        measuredAxes);
                return;
            }
            reason = VisionObservationReason.ACCEPTED_AFTER_GATE_RECOVERY;
        }

        State posterior =
                correct(prior, innovation, observationMatrix, measurementNoise, innovationInverse);

        // Replay everything after the measurement so the correction reaches the present.
        // Corrections
        // already applied at later steps are applied again on top of the new state, so a late frame
        // does not erase newer ones.
        Correction correction =
                new Correction(
                        measurement,
                        variances,
                        measured,
                        reason == VisionObservationReason.ACCEPTED_AFTER_GATE_RECOVERY);
        List<Correction> anchorCorrections = new ArrayList<>(anchor.corrections());
        anchorCorrections.add(correction);
        List<Step> replayed = new ArrayList<>(stepsAfter + 1);
        replayed.add(
                new Step(
                        anchor.timestampSeconds(),
                        anchor.twist(),
                        anchor.dtSeconds(),
                        anchor.tipped(),
                        posterior,
                        anchorCorrections));
        State running = posterior;
        for (Step step : newer) {
            running = predict(running, step.twist(), step.dtSeconds(), step.tipped(), config);
            for (Correction later : step.corrections()) {
                running = reapply(running, later);
            }
            replayed.add(
                    new Step(
                            step.timestampSeconds(),
                            step.twist(),
                            step.dtSeconds(),
                            step.tipped(),
                            running,
                            step.corrections()));
        }

        if (!poseValidator.test(running.pose())) {
            logVision(
                    observation,
                    VisionObservationReason.POSE_VALIDATOR_REJECTED,
                    nis,
                    measuredAxes);
            return;
        }

        for (int i = 0; i <= stepsAfter; i++) {
            history.pollLast();
        }
        history.addAll(replayed);
        if (reason == VisionObservationReason.ACCEPTED_AFTER_GATE_RECOVERY) {
            recentGateOutcomes.clear();
        }
        logVision(observation, reason, nis, measuredAxes);
    }

    /**
     * Applies a previously accepted correction to a new prior during replay. It is not gated again:
     * it was accepted once, and rejecting it now would only let a late frame undo newer vision.
     */
    private State reapply(State prior, Correction c) {
        Matrix<N3, N3> h = observationMatrix(c.measured());
        Matrix<N3, N3> r = measurementNoise(c.variances(), c.measured());
        Vector<N3> innovation = innovation(prior.pose(), c.measurement(), c.measured());
        try {
            State base = prior;
            if (c.recovered()) {
                int axes = 0;
                for (boolean m : c.measured()) {
                    axes += m ? 1 : 0;
                }
                base =
                        new State(
                                prior.pose(),
                                inflateToGate(
                                        prior.covariance(),
                                        innovation,
                                        h,
                                        r,
                                        gateThresholdFor(axes)));
            }
            return correct(base, innovation, h, r);
        } catch (SingularMatrixException e) {
            return prior;
        }
    }

    /**
     * Resets the estimator to a known field-relative pose. Uncertainty is reset to {@link
     * Config#resetLinearStdDev()} and {@link Config#resetAngularStdDev()}, and all history before
     * the reset is discarded, so late vision frames captured before the reset are ignored.
     *
     * @param pose the known pose
     */
    public void resetPose(Pose2d pose) {
        odometry.resetPose(pose);
        recentGateOutcomes.clear();
        seedHistory(pose, config.resetLinearStdDev(), config.resetAngularStdDev());
    }

    /**
     * Sets a validator applied to both odometry and vision-corrected poses.
     *
     * @param validator returns {@code true} for acceptable poses
     * @return this
     */
    public EkfPoseEstimator withPoseValidator(Predicate<Pose2d> validator) {
        poseValidator = validator;
        return this;
    }

    // ---------------------------------------------------------------------------------------
    // EKF math (package-private statics so they can be unit tested)
    // ---------------------------------------------------------------------------------------

    /**
     * Predict step: propagates the pose with {@code Pose2d.exp} and the uncertainty with {@code
     * errorCarryOver · uncertainty · errorCarryOverᵀ + noiseCarryOver · odometryNoise ·
     * noiseCarryOverᵀ}.
     *
     * @param tipped whether the robot is tipped, which adds uncertainty at a fixed rate
     */
    static State predict(
            State prior, Twist2d twist, double dtSeconds, boolean tipped, Config config) {
        double theta = prior.pose().getRotation().getRadians();

        // F·P·Fᵀ + G·Q·Gᵀ written out on scalars. This runs at odometry rate and again for every
        // replayed step of every vision frame, so building Matrix objects here is too costly. The
        // closed form is the same one stateJacobian and noiseJacobian return, which the tests check
        // against finite differences.
        double[] v = arcMatrix(twist.dtheta);
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);

        double localDx = v[0] * twist.dx + v[1] * twist.dy;
        double localDy = v[2] * twist.dx + v[3] * twist.dy;
        double fieldDx = localDx * cos - localDy * sin;
        double fieldDy = localDx * sin + localDy * cos;

        double g00 = cos * v[0] - sin * v[2];
        double g01 = cos * v[1] - sin * v[3];
        double g10 = sin * v[0] + cos * v[2];
        double g11 = sin * v[1] + cos * v[3];

        double distance = Math.hypot(twist.dx, twist.dy);
        double linearVariance =
                config.linearVariancePerMeter() * distance
                        + config.linearVariancePerSecond() * dtSeconds
                        + (tipped ? config.tippedLinearVariancePerSecond() * dtSeconds : 0.0);
        double angularVariance = config.angularVariancePerSecond() * dtSeconds;

        Matrix<N3, N3> p = prior.covariance();
        double p00 = p.get(0, 0);
        double p01 = 0.5 * (p.get(0, 1) + p.get(1, 0));
        double p02 = 0.5 * (p.get(0, 2) + p.get(2, 0));
        double p11 = p.get(1, 1);
        double p12 = 0.5 * (p.get(1, 2) + p.get(2, 1));
        double p22 = p.get(2, 2);

        // F·P, where F = [1 0 −Δy; 0 1 Δx; 0 0 1]
        double a00 = p00 - fieldDy * p02;
        double a01 = p01 - fieldDy * p12;
        double a02 = p02 - fieldDy * p22;
        double a10 = p01 + fieldDx * p02;
        double a11 = p11 + fieldDx * p12;
        double a12 = p12 + fieldDx * p22;

        // (F·P)·Fᵀ, symmetric by construction
        double c00 = a00 - fieldDy * a02;
        double c01 = a01 + fieldDx * a02;
        double c11 = a11 + fieldDx * a12;
        double c02 = a02;
        double c12 = a12;
        double c10 = a10 - fieldDy * a12;

        double n00 = linearVariance * (g00 * g00 + g01 * g01);
        double n01 = linearVariance * (g00 * g10 + g01 * g11);
        double n11 = linearVariance * (g10 * g10 + g11 * g11);

        double r00 = c00 + n00;
        double r01 = 0.5 * (c01 + c10) + n01;
        double r11 = c11 + n11;
        double r22 = p22 + angularVariance;

        Matrix<N3, N3> covariance =
                MatBuilder.fill(Nat.N3(), Nat.N3(), r00, r01, c02, r01, r11, c12, c02, c12, r22);
        return new State(prior.pose().exp(twist), covariance);
    }

    /**
     * Update step. The observation matrix selects the measured axes (a diagonal matrix of ones and
     * zeros), so partial observations such as "position but not heading" are handled exactly,
     * including the correction they imply for correlated unmeasured axes. Uses the Joseph form for
     * the covariance so rounding error cannot make it asymmetric or indefinite.
     *
     * @throws SingularMatrixException if the innovation covariance cannot be inverted
     */
    static State correct(
            State prior,
            Vector<N3> innovation,
            Matrix<N3, N3> observationMatrix,
            Matrix<N3, N3> measurementNoise) {
        return correct(
                prior,
                innovation,
                observationMatrix,
                measurementNoise,
                innovationCovarianceInverse(
                        prior.covariance(), observationMatrix, measurementNoise));
    }

    /**
     * Update step with the inverse of the innovation covariance already computed, so a caller that
     * needed it for the NIS does not invert it twice.
     */
    static State correct(
            State prior,
            Vector<N3> innovation,
            Matrix<N3, N3> observationMatrix,
            Matrix<N3, N3> measurementNoise,
            Matrix<N3, N3> innovationInverse) {
        Matrix<N3, N3> p = prior.covariance();
        Matrix<N3, N3> h = observationMatrix;
        Matrix<N3, N3> gain = p.times(h.transpose()).times(innovationInverse);

        Vector<N3> correction = new Vector<>(gain.times(innovation));
        Pose2d pose =
                new Pose2d(
                        prior.pose().getX() + correction.get(0),
                        prior.pose().getY() + correction.get(1),
                        prior.pose().getRotation().plus(Rotation2d.fromRadians(correction.get(2))));

        Matrix<N3, N3> iMinusKh = Matrix.eye(Nat.N3()).minus(gain.times(h));
        Matrix<N3, N3> covariance =
                iMinusKh.times(p)
                        .times(iMinusKh.transpose())
                        .plus(gain.times(measurementNoise).times(gain.transpose()));

        return new State(pose, symmetrize(covariance));
    }

    /**
     * The gap {@code cameraPose − pose}, in the field frame with the heading wrapped to ±π.
     * Unmeasured axes are zero.
     */
    static Vector<N3> innovation(Pose2d estimate, Pose2d measurement, boolean[] measured) {
        return VecBuilder.fill(
                measured[0] ? measurement.getX() - estimate.getX() : 0.0,
                measured[1] ? measurement.getY() - estimate.getY() : 0.0,
                measured[2]
                        ? MathUtil.angleModulus(
                                measurement.getRotation().getRadians()
                                        - estimate.getRotation().getRadians())
                        : 0.0);
    }

    /**
     * Scales the covariance by the smallest factor (at least 1) for which {@code d² <= threshold}.
     * Used by gate recovery to admit an observation without discarding everything the filter knows:
     * the more the observation disagrees, the more uncertainty is added, and no more.
     *
     * @throws SingularMatrixException if the innovation covariance cannot be inverted
     */
    static Matrix<N3, N3> inflateToGate(
            Matrix<N3, N3> covariance,
            Vector<N3> innovation,
            Matrix<N3, N3> observationMatrix,
            Matrix<N3, N3> measurementNoise,
            double threshold) {
        double low = 1.0;
        double high = 1.0;
        while (high < 1e12
                && normalizedInnovationSquared(
                                innovation,
                                covariance.times(high),
                                observationMatrix,
                                measurementNoise)
                        > threshold) {
            low = high;
            high *= 2.0;
        }
        for (int i = 0; i < 40; i++) {
            double mid = 0.5 * (low + high);
            double nis =
                    normalizedInnovationSquared(
                            innovation, covariance.times(mid), observationMatrix, measurementNoise);
            if (nis > threshold) {
                low = mid;
            } else {
                high = mid;
            }
        }
        return covariance.times(high);
    }

    /** Diagonal matrix with a 1 for each measured axis and a 0 for the others. */
    static Matrix<N3, N3> observationMatrix(boolean[] measured) {
        return diagonal(measured[0] ? 1.0 : 0.0, measured[1] ? 1.0 : 0.0, measured[2] ? 1.0 : 0.0);
    }

    /**
     * Camera uncertainty: the variance for each measured axis. Unmeasured axes get an arbitrary 1.0
     * on the diagonal, which has no effect because the observation matrix has a zero row there and
     * the gap is zero, but keeps the expected gap invertible.
     */
    static Matrix<N3, N3> measurementNoise(double[] variances, boolean[] measured) {
        return diagonal(
                measured[0] ? variances[0] : 1.0,
                measured[1] ? variances[1] : 1.0,
                measured[2] ? variances[2] : 1.0);
    }

    /**
     * NIS of a vision frame: {@code gapᵀ · expectedGap⁻¹ · gap}, where the expected gap is the
     * filter's uncertainty on the measured axes plus the camera uncertainty. Chi-squared with one
     * degree of freedom per measured axis if the filter is consistent.
     *
     * @throws SingularMatrixException if the expected gap cannot be inverted
     */
    static double normalizedInnovationSquared(
            Vector<N3> innovation,
            Matrix<N3, N3> covariance,
            Matrix<N3, N3> observationMatrix,
            Matrix<N3, N3> measurementNoise) {
        return quadraticForm(
                innovation,
                innovationCovarianceInverse(covariance, observationMatrix, measurementNoise));
    }

    /**
     * Inverse of the expected gap {@code H·P·Hᵀ + R}.
     *
     * @throws SingularMatrixException if it cannot be inverted
     */
    static Matrix<N3, N3> innovationCovarianceInverse(
            Matrix<N3, N3> covariance,
            Matrix<N3, N3> observationMatrix,
            Matrix<N3, N3> measurementNoise) {
        Matrix<N3, N3> h = observationMatrix;
        return h.times(covariance).times(h.transpose()).plus(measurementNoise).inv();
    }

    private static double quadraticForm(Vector<N3> innovation, Matrix<N3, N3> inverse) {
        return innovation.transpose().times(inverse).times(innovation).get(0, 0);
    }

    /**
     * Rejection threshold on the NIS for {@code dof} measured axes.
     *
     * <p>{@link Config#gateChiSquared()} is specified for 3 degrees of freedom. A chi-squared
     * variable with fewer degrees of freedom is smaller on average (its mean is the number of
     * degrees of freedom), so reusing the 3-DOF number for a 2-axis observation would gate too
     * loosely. Instead this finds the threshold that rejects the same fraction of good
     * observations: it computes the tail probability beyond the 3-DOF number, then solves for the
     * threshold with that same tail probability at {@code dof} degrees of freedom. For example
     * 16.27 (0.1% at 3 DOF) becomes 13.82 at 2 DOF and 10.83 at 1 DOF.
     *
     * @param dof number of measured axes, 1 to 3
     * @param gateChiSquared the 3-DOF threshold, or {@code +∞} to disable gating
     */
    static double gateThreshold(int dof, double gateChiSquared) {
        if (dof == 3 || Double.isInfinite(gateChiSquared)) {
            return gateChiSquared;
        }
        double tailProbability = 1.0 - chiSquaredCdf(3, gateChiSquared);
        double low = 0.0;
        double high = 1000.0;
        for (int i = 0; i < 100; i++) {
            double mid = 0.5 * (low + high);
            if (1.0 - chiSquaredCdf(dof, mid) > tailProbability) {
                low = mid;
            } else {
                high = mid;
            }
        }
        return 0.5 * (low + high);
    }

    /** Chi-squared CDF in closed form for 1, 2 and 3 degrees of freedom. */
    private static double chiSquaredCdf(int dof, double x) {
        if (x <= 0.0) {
            return 0.0;
        }
        return switch (dof) {
            case 1 -> erf(Math.sqrt(x / 2.0));
            case 2 -> 1.0 - Math.exp(-x / 2.0);
            case 3 -> erf(Math.sqrt(x / 2.0)) - Math.sqrt(2.0 * x / Math.PI) * Math.exp(-x / 2.0);
            default -> throw new IllegalArgumentException("dof must be 1, 2 or 3, got " + dof);
        };
    }

    /** Error function for {@code x >= 0}, Abramowitz and Stegun 7.1.26 (error below 1.5e-7). */
    private static double erf(double x) {
        double t = 1.0 / (1.0 + 0.3275911 * x);
        double polynomial = 1.061405429;
        polynomial = polynomial * t - 1.453152027;
        polynomial = polynomial * t + 1.421413741;
        polynomial = polynomial * t - 0.284496736;
        polynomial = polynomial * t + 0.254829592;
        polynomial *= t;
        return 1.0 - polynomial * Math.exp(-x * x);
    }

    private static boolean allFinite(double[] values) {
        for (double value : values) {
            if (!Double.isFinite(value)) {
                return false;
            }
        }
        return true;
    }

    /**
     * The error carry-over {@code ∂f/∂pose} for {@code f(pose) = pose.exp(twist)} at heading {@code
     * theta}. The third column is the field-frame displacement rotated 90°, which is how heading
     * error becomes position error.
     */
    static Matrix<N3, N3> stateJacobian(double theta, Twist2d twist) {
        double[] local = arcTranslation(twist);
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);
        double fieldDx = local[0] * cos - local[1] * sin;
        double fieldDy = local[0] * sin + local[1] * cos;
        return MatBuilder.fill(
                Nat.N3(), Nat.N3(), 1.0, 0.0, -fieldDy, 0.0, 1.0, fieldDx, 0.0, 0.0, 1.0);
    }

    /**
     * The noise carry-over {@code ∂f/∂twist}. The translation block is {@code rotation(heading) ·
     * V(dheading)}. The dependence of the translation on the heading change is neglected, which is
     * accurate for the few milliseconds between odometry samples.
     */
    static Matrix<N3, N3> noiseJacobian(double theta, Twist2d twist) {
        double[] v = arcMatrix(twist.dtheta);
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);
        // R(θ) * V
        double m00 = cos * v[0] - sin * v[2];
        double m01 = cos * v[1] - sin * v[3];
        double m10 = sin * v[0] + cos * v[2];
        double m11 = sin * v[1] + cos * v[3];
        return MatBuilder.fill(Nat.N3(), Nat.N3(), m00, m01, 0.0, m10, m11, 0.0, 0.0, 0.0, 1.0);
    }

    /** Robot-frame translation of {@code Pose2d.exp(twist)}: {@code V(dθ)·[dx, dy]}. */
    private static double[] arcTranslation(Twist2d twist) {
        double[] v = arcMatrix(twist.dtheta);
        return new double[] {v[0] * twist.dx + v[1] * twist.dy, v[2] * twist.dx + v[3] * twist.dy};
    }

    /**
     * Row-major {@code V(dθ) = [[s, −c], [c, s]]} with {@code s = sin(dθ)/dθ, c = (1−cos dθ)/dθ}.
     */
    private static double[] arcMatrix(double dtheta) {
        double s;
        double c;
        if (Math.abs(dtheta) < ARC_EPSILON) {
            s = 1.0 - dtheta * dtheta / 6.0;
            c = dtheta / 2.0;
        } else {
            s = Math.sin(dtheta) / dtheta;
            c = (1.0 - Math.cos(dtheta)) / dtheta;
        }
        return new double[] {s, -c, c, s};
    }

    private static Matrix<N3, N3> diagonal(double a, double b, double c) {
        return MatBuilder.fill(Nat.N3(), Nat.N3(), a, 0.0, 0.0, 0.0, b, 0.0, 0.0, 0.0, c);
    }

    private static Matrix<N3, N3> symmetrize(Matrix<N3, N3> m) {
        return m.plus(m.transpose()).times(0.5);
    }

    private static double square(double value) {
        return value * value;
    }

    // ---------------------------------------------------------------------------------------
    // Internals
    // ---------------------------------------------------------------------------------------

    private Step latest() {
        return history.getLast();
    }

    private void seedHistory(Pose2d pose, double linearStdDev, double angularStdDev) {
        history.clear();
        double timestamp =
                Double.isNaN(lastTimestampSeconds)
                        ? Double.NEGATIVE_INFINITY
                        : lastTimestampSeconds;
        history.add(
                new Step(
                        timestamp,
                        new Twist2d(),
                        0.0,
                        false,
                        new State(
                                pose,
                                diagonal(
                                        square(linearStdDev),
                                        square(linearStdDev),
                                        square(angularStdDev))),
                        List.of()));
    }

    private void predictFromOdometry(Pose2d lastOdometryPose, double timestampSeconds) {
        applyOdometryTwist(lastOdometryPose.log(odometry.odometryPose()), timestampSeconds);
    }

    /**
     * Runs the predict step for one odometry sample. Package-private so tests can drive the filter
     * from a recorded stream of twists.
     */
    void applyOdometryTwist(Twist2d twist, double timestampSeconds) {
        if (!Double.isFinite(twist.dx)
                || !Double.isFinite(twist.dy)
                || !Double.isFinite(twist.dtheta)) {
            // Bad module or gyro data must not poison the covariance with NaN
            Logger.recordOutput(LOG_PREFIX + "Odometry/NonFiniteTwistSkipped", true);
            return;
        }
        double dt =
                Double.isNaN(lastTimestampSeconds)
                        ? 0.0
                        : Math.max(0.0, timestampSeconds - lastTimestampSeconds);
        lastTimestampSeconds = timestampSeconds;

        State next = predict(latest().state(), twist, dt, tipped, config);
        history.add(new Step(timestampSeconds, twist, dt, tipped, next, List.of()));

        // Drop history older than the buffer, but always keep the most recent step
        double cutoff = timestampSeconds - bufferSeconds;
        while (history.size() > 2) {
            Step oldest = history.pollFirst();
            if (history.peekFirst().timestampSeconds() >= cutoff) {
                history.addFirst(oldest);
                break;
            }
        }
    }

    private void recordGateOutcome(double timestampSeconds, boolean gated) {
        recentGateOutcomes.addLast(new GateOutcome(timestampSeconds, gated));
        double oldest = timestampSeconds - config.gateRecoveryWindowSeconds();
        recentGateOutcomes.removeIf(outcome -> outcome.timestampSeconds() < oldest);
    }

    private boolean shouldRecoverFromGating() {
        long rejected = recentGateOutcomes.stream().filter(GateOutcome::gated).count();
        return rejected >= config.gateRecoveryCount()
                && rejected >= config.gateRecoveryFraction() * recentGateOutcomes.size();
    }

    private double gateThresholdFor(int dof) {
        double chiSquared = config.gateChiSquared();
        if (chiSquared != cachedGateChiSquared) {
            Arrays.fill(cachedGateThresholds, Double.NaN);
            cachedGateChiSquared = chiSquared;
        }
        double threshold = cachedGateThresholds[dof];
        if (Double.isNaN(threshold)) {
            threshold = gateThreshold(dof, chiSquared);
            cachedGateThresholds[dof] = threshold;
        }
        return threshold;
    }

    /**
     * How many steps are newer than the latest step at or before {@code timestampSeconds}, or -1 if
     * there is no step at or before it. Walks back from the newest step, so the cost follows the
     * age of the timestamp and not the length of the buffer.
     */
    private int stepsAfter(double timestampSeconds) {
        int newer = 0;
        for (Iterator<Step> it = history.descendingIterator(); it.hasNext(); ) {
            if (it.next().timestampSeconds() <= timestampSeconds) {
                return newer;
            }
            newer++;
        }
        return -1;
    }

    private void logVision(
            VisionPoseObservation observation,
            VisionObservationReason reason,
            double nis,
            int measuredAxes) {
        lastVisionReason = reason;
        lastNis = nis;
        boolean accepted =
                reason == VisionObservationReason.ACCEPTED
                        || reason == VisionObservationReason.ACCEPTED_AFTER_GATE_RECOVERY;
        Matrix<N3, N3> p = covariance();

        Logger.recordOutput(LOG_PREFIX + "Vision/Accepted", accepted);
        Logger.recordOutput(LOG_PREFIX + "Vision/Reason", reason.name());
        Logger.recordOutput(LOG_PREFIX + "Vision/ObservationPose", observation.robotPose());
        Logger.recordOutput(LOG_PREFIX + "Vision/TimestampSeconds", observation.timestampSeconds());
        Logger.recordOutput(LOG_PREFIX + "Vision/LinearStdDev", observation.linearStdDev());
        Logger.recordOutput(LOG_PREFIX + "Vision/AngularStdDev", observation.angularStdDev());
        Logger.recordOutput(LOG_PREFIX + "Vision/Nis", nis);
        Logger.recordOutput(LOG_PREFIX + "Vision/MeasuredAxes", measuredAxes);
        Logger.recordOutput(
                LOG_PREFIX + "Vision/GateChiSquared",
                measuredAxes == 0 ? config.gateChiSquared() : gateThresholdFor(measuredAxes));
        Logger.recordOutput(
                LOG_PREFIX + "Vision/RecentGateRejections",
                recentGateOutcomes.stream().filter(GateOutcome::gated).count());
        Logger.recordOutput(LOG_PREFIX + "Covariance/SigmaX", Math.sqrt(p.get(0, 0)));
        Logger.recordOutput(LOG_PREFIX + "Covariance/SigmaY", Math.sqrt(p.get(1, 1)));
        Logger.recordOutput(LOG_PREFIX + "Covariance/SigmaTheta", Math.sqrt(p.get(2, 2)));
    }
}
