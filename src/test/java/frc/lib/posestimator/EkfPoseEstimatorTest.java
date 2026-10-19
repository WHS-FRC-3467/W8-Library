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

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;

import frc.lib.posestimator.EkfPoseEstimator.Config;
import frc.lib.posestimator.PoseEstimator.VisionPoseObservation;
import frc.lib.posestimator.SwerveOdometry.OdometryObservation;

import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Optional;

class EkfPoseEstimatorTest {
    private static final double ODOMETRY_PERIOD = 0.01;

    private static final SwerveDriveKinematics KINEMATICS =
            new SwerveDriveKinematics(
                    new Translation2d(0.3, 0.3),
                    new Translation2d(0.3, -0.3),
                    new Translation2d(-0.3, 0.3),
                    new Translation2d(-0.3, -0.3));

    // ---- helpers ----

    /** Odometry for a robot driving straight along +x with the given total distance. */
    private static OdometryObservation odometry(double timestamp, double distance) {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = new SwerveModulePosition(distance, Rotation2d.kZero);
        }
        return new OdometryObservation(
                Seconds.of(timestamp), positions, Optional.of(Rotation2d.kZero));
    }

    private static VisionPoseObservation vision(
            double timestamp, Pose2d pose, double linearStdDev, double angularStdDev) {
        return new VisionPoseObservation(
                timestamp, pose, 2.0, List.of(1), linearStdDev, angularStdDev);
    }

    /** Drives straight at 1 m/s from {@code startTime} (inclusive) to {@code endTime}. */
    private static void drive(EkfPoseEstimator estimator, double startTime, double endTime) {
        int steps = (int) Math.round((endTime - startTime) / ODOMETRY_PERIOD);
        for (int i = 0; i <= steps; i++) {
            double t = startTime + i * ODOMETRY_PERIOD;
            estimator.addOdometryObservation(odometry(t, t));
        }
    }

    private static double sigma(EkfPoseEstimator estimator, int axis) {
        return Math.sqrt(estimator.covariance().get(axis, axis));
    }

    // ---- Jacobians ----

    @Test
    void stateJacobianMatchesFiniteDifference() {
        double[] thetas = {0.0, 0.7, -2.1, 3.0};
        Twist2d[] twists = {
            new Twist2d(0.05, 0.0, 0.0),
            new Twist2d(0.04, -0.03, 0.02),
            new Twist2d(0.3, 0.2, -0.4),
            new Twist2d(0.0, 0.0, 0.1)
        };
        double eps = 1e-6;

        for (double theta : thetas) {
            for (Twist2d twist : twists) {
                Matrix<N3, N3> analytic = EkfPoseEstimator.stateJacobian(theta, twist);
                double[] base = {1.5, -0.5, theta};

                for (int col = 0; col < 3; col++) {
                    double[] plus = base.clone();
                    double[] minus = base.clone();
                    plus[col] += eps;
                    minus[col] -= eps;
                    double[] fPlus = propagate(plus, twist);
                    double[] fMinus = propagate(minus, twist);
                    for (int row = 0; row < 3; row++) {
                        double numeric = (fPlus[row] - fMinus[row]) / (2 * eps);
                        assertEquals(
                                numeric,
                                analytic.get(row, col),
                                1e-6,
                                "F[" + row + "][" + col + "] theta=" + theta);
                    }
                }
            }
        }
    }

    @Test
    void noiseJacobianMatchesFiniteDifferenceForTranslationInputs() {
        double eps = 1e-6;
        for (double theta : new double[] {0.0, 1.1, -2.5}) {
            Twist2d twist = new Twist2d(0.04, -0.03, 0.02);
            Matrix<N3, N3> analytic = EkfPoseEstimator.noiseJacobian(theta, twist);
            double[] base = {1.5, -0.5, theta};

            // dx and dy enter linearly, so their columns are exact
            for (int col = 0; col < 2; col++) {
                Twist2d plus =
                        new Twist2d(
                                twist.dx + (col == 0 ? eps : 0),
                                twist.dy + (col == 1 ? eps : 0),
                                twist.dtheta);
                Twist2d minus =
                        new Twist2d(
                                twist.dx - (col == 0 ? eps : 0),
                                twist.dy - (col == 1 ? eps : 0),
                                twist.dtheta);
                double[] fPlus = propagate(base, plus);
                double[] fMinus = propagate(base, minus);
                for (int row = 0; row < 3; row++) {
                    assertEquals(
                            (fPlus[row] - fMinus[row]) / (2 * eps),
                            analytic.get(row, col),
                            1e-6,
                            "G[" + row + "][" + col + "] theta=" + theta);
                }
            }
            // dθ adds directly to heading
            assertEquals(1.0, analytic.get(2, 2), 1e-12);
        }
    }

    private static double[] propagate(double[] state, Twist2d twist) {
        Pose2d next = new Pose2d(state[0], state[1], Rotation2d.fromRadians(state[2])).exp(twist);
        // Unwrapped heading so finite differences do not see a 2π jump
        return new double[] {next.getX(), next.getY(), state[2] + twist.dtheta};
    }

    // ---- covariance behaviour ----

    @Test
    void covarianceGrowsWhileDrivingAndStaysSymmetricPositive() {
        EkfPoseEstimator estimator = new EkfPoseEstimator(KINEMATICS);
        estimator.resetPose(new Pose2d(0, 0, Rotation2d.kZero));
        double before = sigma(estimator, 0);

        drive(estimator, 0.0, 10.0);

        assertTrue(sigma(estimator, 0) > before * 2.0, "x uncertainty should grow with distance");
        assertTrue(sigma(estimator, 2) > Config.DEFAULT.resetAngularStdDev());

        Matrix<N3, N3> p = estimator.covariance();
        for (int i = 0; i < 3; i++) {
            assertTrue(p.get(i, i) > 0.0);
            for (int j = 0; j < 3; j++) {
                assertEquals(p.get(i, j), p.get(j, i), 1e-12);
            }
        }
    }

    @Test
    void headingUncertaintyLeaksIntoPositionWhileMoving() {
        // Drive along +x with heading uncertainty: y should become uncertain and correlate with θ
        // even though odometry noise on y alone is tiny.
        Config config =
                Config.DEFAULT.toBuilder()
                        .linearVariancePerMeter(0.0)
                        .linearVariancePerSecond(0.0)
                        .angularVariancePerSecond(0.0)
                        .resetLinearStdDev(0.0)
                        .resetAngularStdDev(0.1)
                        .build();
        EkfPoseEstimator estimator = new EkfPoseEstimator(KINEMATICS, config);
        estimator.resetPose(new Pose2d(0, 0, Rotation2d.kZero));

        drive(estimator, 0.0, 5.0);

        // y ≈ θ * distance, so σy ≈ 0.1 * 5 = 0.5
        assertEquals(0.5, sigma(estimator, 1), 0.02);
        assertTrue(estimator.covariance().get(1, 2) > 0.0, "y and θ errors must be correlated");
    }

    // ---- vision ----

    @Test
    void visionPullsEstimateAndShrinksCovariance() {
        Config config = Config.DEFAULT.toBuilder().resetLinearStdDev(0.5).build();
        EkfPoseEstimator estimator = new EkfPoseEstimator(KINEMATICS, config);
        estimator.resetPose(new Pose2d(0.5, 0.0, Rotation2d.kZero)); // truth starts at x=0

        drive(estimator, 0.0, 1.0);
        double sigmaBefore = sigma(estimator, 0);

        // Truth at t=1 is x=1, estimate is 1.5
        estimator.addVisionObservation(
                vision(1.0, new Pose2d(1.0, 0, Rotation2d.kZero), 0.05, 0.02));

        assertEquals(1.0, estimator.estimatedPose().getX(), 0.05);
        assertTrue(sigma(estimator, 0) < sigmaBefore);
    }

    @Test
    void lateVisionFrameIsReplayedToThePresent() {
        Config config = Config.DEFAULT.toBuilder().resetLinearStdDev(0.5).build();
        EkfPoseEstimator estimator = new EkfPoseEstimator(KINEMATICS, config);
        estimator.resetPose(new Pose2d(0, 0, Rotation2d.kZero));

        drive(estimator, 0.0, 1.0);

        // A frame captured at t=0.5 says we were really at x=0.8 (estimate said 0.5). The robot
        // then drove 0.5 m more, so the corrected present pose is about 1.3, not 0.8.
        estimator.addVisionObservation(
                vision(0.5, new Pose2d(0.8, 0, Rotation2d.kZero), 0.001, 0.001));

        assertEquals(1.3, estimator.estimatedPose().getX(), 0.01);
        // History is corrected too
        Optional<Pose2d> pastPose = estimator.getPoseAtTime(0.5);
        assertTrue(pastPose.isPresent());
        assertEquals(0.8, pastPose.get().getX(), 0.01);
    }

    @Test
    void visionOlderThanRetainedHistoryIsIgnored() {
        EkfPoseEstimator estimator = new EkfPoseEstimator(KINEMATICS);
        estimator.resetPose(new Pose2d(0, 0, Rotation2d.kZero));
        drive(estimator, 10.0, 15.0); // buffer is 2 s, so t=10 has been dropped

        Pose2d before = estimator.estimatedPose();
        estimator.addVisionObservation(
                vision(10.0, new Pose2d(5, 5, Rotation2d.kZero), 0.01, 0.01));

        assertEquals(before, estimator.estimatedPose());
        assertTrue(estimator.getPoseAtTime(10.0).isEmpty());
    }

    // ---- unmeasured axes and bad observations ----

    @Test
    void infiniteAngularStdDevCorrectsPositionAndLeavesHeadingAlone() {
        // VisionSubsystem reports +∞ angular std dev for single-tag solves ("ignore rotation")
        Config config = Config.DEFAULT.toBuilder().resetLinearStdDev(0.5).build();
        EkfPoseEstimator estimator = new EkfPoseEstimator(KINEMATICS, config);
        estimator.resetPose(new Pose2d(0.5, 0.0, Rotation2d.kZero));
        drive(estimator, 0.0, 1.0);
        double headingSigmaBefore = sigma(estimator, 2);

        // Vision heading is garbage here, which must not leak in
        Pose2d measurement = new Pose2d(1.0, 0.0, Rotation2d.fromDegrees(90));
        estimator.addVisionObservation(vision(1.0, measurement, 0.05, Double.POSITIVE_INFINITY));

        assertEquals(1.0, estimator.estimatedPose().getX(), 0.05);
        assertEquals(0.0, estimator.estimatedPose().getRotation().getRadians(), 1e-6);
        // Position-only measurements can only reduce heading uncertainty, never add to it
        assertTrue(sigma(estimator, 2) <= headingSigmaBefore + 1e-12);
        assertTrue(Double.isFinite(sigma(estimator, 0)));
    }

    @Test
    void unmeasuredHeadingStillCorrectsThroughCorrelation() {
        // Heading is uncertain and correlated with y after driving, so a pure position measurement
        // should still shrink the heading uncertainty even though it never measures heading.
        Config config =
                Config.DEFAULT.toBuilder()
                        .linearVariancePerMeter(0.0)
                        .linearVariancePerSecond(0.0)
                        .angularVariancePerSecond(0.0)
                        .resetLinearStdDev(0.0)
                        .resetAngularStdDev(0.1)
                        .build();
        EkfPoseEstimator estimator = new EkfPoseEstimator(KINEMATICS, config);
        estimator.resetPose(new Pose2d(0, 0, Rotation2d.kZero));
        drive(estimator, 0.0, 5.0);
        double headingSigmaBefore = sigma(estimator, 2);

        estimator.addVisionObservation(
                vision(5.0, estimator.estimatedPose(), 0.02, Double.POSITIVE_INFINITY));

        assertTrue(sigma(estimator, 2) < headingSigmaBefore * 0.5);
    }

    @Test
    void nonFiniteObservationsAreIgnoredInsteadOfThrowing() {
        EkfPoseEstimator estimator = new EkfPoseEstimator(KINEMATICS);
        estimator.resetPose(new Pose2d(0, 0, Rotation2d.kZero));
        drive(estimator, 0.0, 1.0);
        Pose2d before = estimator.estimatedPose();
        Matrix<N3, N3> covarianceBefore = estimator.covariance();

        double nan = Double.NaN;
        double inf = Double.POSITIVE_INFINITY;
        Pose2d good = new Pose2d(before.getX() + 0.01, 0, Rotation2d.kZero);
        List<VisionPoseObservation> bad =
                List.of(
                        vision(1.0, good, nan, 0.1), // NaN std dev
                        vision(1.0, good, inf, inf), // nothing measured at all
                        vision(1.0, new Pose2d(nan, 0, Rotation2d.kZero), 0.05, 0.1),
                        vision(1.0, new Pose2d(0, inf, Rotation2d.kZero), 0.05, 0.1));
        for (VisionPoseObservation observation : bad) {
            estimator.addVisionObservation(observation);
        }

        assertEquals(before, estimator.estimatedPose());
        assertEquals(covarianceBefore, estimator.covariance());
    }

    @Test
    void nonFiniteOdometryDoesNotPoisonTheCovariance() {
        EkfPoseEstimator estimator = new EkfPoseEstimator(KINEMATICS);
        estimator.resetPose(new Pose2d(0, 0, Rotation2d.kZero));
        drive(estimator, 0.0, 1.0);
        Pose2d before = estimator.estimatedPose();

        estimator.addOdometryObservation(odometry(1.01, Double.NaN));

        assertEquals(before, estimator.estimatedPose());
        Matrix<N3, N3> p = estimator.covariance();
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                assertTrue(Double.isFinite(p.get(i, j)));
            }
        }
    }

    @Test
    void singularInnovationCovarianceIsIgnoredInsteadOfThrowing() {
        // Perfectly known state and a perfect measurement: S = 0 is not invertible
        Config config =
                Config.DEFAULT.toBuilder().resetLinearStdDev(0.0).resetAngularStdDev(0.0).build();
        EkfPoseEstimator estimator = new EkfPoseEstimator(KINEMATICS, config);
        estimator.resetPose(new Pose2d(0, 0, Rotation2d.kZero));
        Pose2d before = estimator.estimatedPose();

        estimator.addVisionObservation(vision(0.0, new Pose2d(1, 1, Rotation2d.kZero), 0.0, 0.0));

        assertEquals(before, estimator.estimatedPose());
    }

    @Test
    void gateThresholdMatchesChiSquaredQuantilesForEachDegreeOfFreedom() {
        // Same tail probability (0.1%) as the 3-DOF default, expressed for fewer measured axes
        assertEquals(16.27, EkfPoseEstimator.gateThreshold(3, 16.27), 1e-9);
        assertEquals(13.82, EkfPoseEstimator.gateThreshold(2, 16.27), 0.05);
        assertEquals(10.83, EkfPoseEstimator.gateThreshold(1, 16.27), 0.05);
        assertEquals(
                Double.POSITIVE_INFINITY,
                EkfPoseEstimator.gateThreshold(2, Double.POSITIVE_INFINITY));
    }

    // ---- gating ----

    @Test
    void outlierIsRejectedThenRecoveredAfterRepeatedDisagreement() {
        Config config = Config.DEFAULT.toBuilder().gateRecoveryCount(3).build();
        EkfPoseEstimator estimator = new EkfPoseEstimator(KINEMATICS, config);
        estimator.resetPose(new Pose2d(0, 0, Rotation2d.kZero));
        drive(estimator, 0.0, 1.0);

        Pose2d before = estimator.estimatedPose();
        Pose2d far = new Pose2d(before.getX() + 5.0, 3.0, Rotation2d.kZero);

        // First two are gated and leave the estimate alone
        for (int i = 0; i < 2; i++) {
            estimator.addVisionObservation(vision(1.0, far, 0.05, 0.02));
            assertEquals(before, estimator.estimatedPose(), "gated observation must not move pose");
        }

        // The third triggers recovery: covariance is re-inflated and vision is accepted
        estimator.addVisionObservation(vision(1.0, far, 0.05, 0.02));
        assertTrue(
                estimator.estimatedPose().getTranslation().getDistance(far.getTranslation()) < 0.5,
                "recovery should pull the estimate to vision");
    }

    /** Whether any of the observations was admitted through gate recovery. */
    private static boolean feed(EkfPoseEstimator estimator, List<VisionPoseObservation> sequence) {
        boolean recovered = false;
        for (VisionPoseObservation observation : sequence) {
            estimator.addVisionObservation(observation);
            if (estimator.lastVisionReason
                    == EkfPoseEstimator.VisionObservationReason.ACCEPTED_AFTER_GATE_RECOVERY) {
                recovered = true;
            }
        }
        return recovered;
    }

    @Test
    void lowQualityFramesCannotHoldTheFilterInTheWrongPlace() {
        // Reproduces a field failure: the filter was wrong and overconfident, so every high-quality
        // frame was gated, while an occasional accepted low-quality frame kept resetting a
        // consecutive-rejection counter and recovery never happened.
        Config config =
                Config.DEFAULT.toBuilder()
                        .linearVariancePerMeter(0.0)
                        .linearVariancePerSecond(0.0)
                        .angularVariancePerSecond(0.0)
                        .resetLinearStdDev(0.05)
                        .gateRecoveryCount(6)
                        .build();
        EkfPoseEstimator estimator = new EkfPoseEstimator(KINEMATICS, config);
        estimator.resetPose(new Pose2d(0, 0, Rotation2d.kZero));

        Pose2d truth = new Pose2d(1.0, 0, Rotation2d.kZero);
        VisionPoseObservation good = vision(0.0, truth, 0.05, 0.02);
        VisionPoseObservation weak = vision(0.0, truth, 0.8, 0.5);
        List<VisionPoseObservation> sequence =
                List.of(good, good, weak, good, good, weak, good, good, weak, good, good, weak);

        assertTrue(feed(estimator, sequence), "sustained disagreement must trigger recovery");
        assertEquals(1.0, estimator.estimatedPose().getX(), 0.1);
    }

    @Test
    void rejectionsOutsideTheRecoveryWindowDoNotAddUp() {
        Config config =
                Config.DEFAULT.toBuilder()
                        .resetLinearStdDev(0.05)
                        .gateRecoveryCount(3)
                        .gateRecoveryWindowSeconds(1.0)
                        .build();
        EkfPoseEstimator estimator = new EkfPoseEstimator(KINEMATICS, config);
        estimator.resetPose(new Pose2d(0, 0, Rotation2d.kZero));

        Pose2d far = new Pose2d(5, 0, Rotation2d.kZero);
        // One rejection every 2 s: never 3 inside a 1 s window
        List<VisionPoseObservation> sequence =
                List.of(
                        vision(0, far, 0.05, 0.02),
                        vision(2, far, 0.05, 0.02),
                        vision(4, far, 0.05, 0.02),
                        vision(6, far, 0.05, 0.02),
                        vision(8, far, 0.05, 0.02));

        assertFalse(feed(estimator, sequence));
        assertEquals(0.0, estimator.estimatedPose().getX(), 1e-9);
    }

    @Test
    void occasionalOutliersAmongGoodFramesDoNotTriggerRecovery() {
        Config config =
                Config.DEFAULT.toBuilder()
                        .resetLinearStdDev(0.05)
                        .gateRecoveryCount(3)
                        .gateRecoveryFraction(0.5)
                        .build();
        EkfPoseEstimator estimator = new EkfPoseEstimator(KINEMATICS, config);
        estimator.resetPose(new Pose2d(0, 0, Rotation2d.kZero));

        VisionPoseObservation good = vision(0.0, new Pose2d(0, 0, Rotation2d.kZero), 0.05, 0.02);
        VisionPoseObservation outlier = vision(0.0, new Pose2d(5, 0, Rotation2d.kZero), 0.05, 0.02);
        // Three outliers (the count) but only 3 of 15 observations (below the fraction)
        List<VisionPoseObservation> sequence =
                List.of(
                        good, good, good, good, outlier, good, good, good, good, outlier, good,
                        good, good, good, outlier);

        assertFalse(feed(estimator, sequence));
        assertEquals(0.0, estimator.estimatedPose().getX(), 1e-9);
        assertEquals(
                EkfPoseEstimator.VisionObservationReason.GATED_OUTLIER, estimator.lastVisionReason);
    }

    @Test
    void gateRecoveryInflatesCovarianceByTheLeastAmountThatAdmitsTheFrame() {
        Matrix<N3, N3> covariance = Matrix.eye(Nat.N3()).times(0.0025);
        Matrix<N3, N3> noise = Matrix.eye(Nat.N3()).times(0.0025);
        Matrix<N3, N3> observation = Matrix.eye(Nat.N3());
        double threshold = 16.27;

        // A 0.5 m disagreement is far outside what sigma = 5 cm allows
        var big = VecBuilder.fill(0.5, 0.0, 0.0);
        assertTrue(
                EkfPoseEstimator.normalizedInnovationSquared(big, covariance, observation, noise)
                        > threshold);
        Matrix<N3, N3> inflated =
                EkfPoseEstimator.inflateToGate(covariance, big, observation, noise, threshold);
        double nis =
                EkfPoseEstimator.normalizedInnovationSquared(big, inflated, observation, noise);
        assertTrue(nis <= threshold, "inflated covariance must admit the observation");
        assertTrue(nis > 0.99 * threshold, "and add no more uncertainty than needed, got " + nis);
        assertTrue(inflated.get(0, 0) > covariance.get(0, 0));

        // Already inside the gate: nothing to add
        var small = VecBuilder.fill(0.01, 0.0, 0.0);
        assertEquals(
                covariance,
                EkfPoseEstimator.inflateToGate(covariance, small, observation, noise, threshold));
    }

    @Test
    void gatingCanBeDisabled() {
        Config config = Config.DEFAULT.toBuilder().gateChiSquared(Double.POSITIVE_INFINITY).build();
        EkfPoseEstimator estimator = new EkfPoseEstimator(KINEMATICS, config);
        estimator.resetPose(new Pose2d(0, 0, Rotation2d.kZero));
        drive(estimator, 0.0, 1.0);

        Pose2d before = estimator.estimatedPose();
        estimator.addVisionObservation(vision(1.0, new Pose2d(5, 3, Rotation2d.kZero), 0.05, 0.02));

        assertFalse(before.equals(estimator.estimatedPose()));
    }

    @Test
    void poseValidatorRejectsVisionCorrection() {
        Config config = Config.DEFAULT.toBuilder().resetLinearStdDev(0.5).build();
        EkfPoseEstimator estimator =
                new EkfPoseEstimator(KINEMATICS, config)
                        .withPoseValidator(pose -> pose.getX() < 3.0);
        estimator.resetPose(new Pose2d(0, 0, Rotation2d.kZero));
        drive(estimator, 0.0, 1.0);

        Pose2d before = estimator.estimatedPose();
        estimator.addVisionObservation(
                vision(1.0, new Pose2d(4, 0, Rotation2d.kZero), 0.001, 0.001));

        assertEquals(before, estimator.estimatedPose());
    }

    // ---- config / reset ----

    // ---- tipped ----

    /** Odometry steps are multiples of this, a power of two so timestamps are exact. */
    private static final double TIP_STEP = 0.125;

    private static void stepStationary(EkfPoseEstimator estimator, int first, int last) {
        for (int k = first; k <= last; k++) {
            estimator.applyOdometryTwist(new Twist2d(), k * TIP_STEP);
        }
    }

    private static double pull(EkfPoseEstimator estimator) {
        return estimator.estimatedPose().getX();
    }

    @Test
    void uncertaintyGrowsWhileTippedSoTheNextFramesPullHarder() {
        Config config =
                Config.DEFAULT.toBuilder()
                        .resetLinearStdDev(0.05)
                        .linearVariancePerSecond(0.0)
                        .build();
        EkfPoseEstimator level = new EkfPoseEstimator(KINEMATICS, config);
        EkfPoseEstimator wasTipped = new EkfPoseEstimator(KINEMATICS, config);
        level.resetPose(new Pose2d(0, 0, Rotation2d.kZero));
        wasTipped.resetPose(new Pose2d(0, 0, Rotation2d.kZero));
        stepStationary(level, 1, 16);
        wasTipped.setTipped(true);
        stepStationary(wasTipped, 1, 8); // tipped for the first second
        wasTipped.setTipped(false);
        stepStationary(wasTipped, 9, 16);

        // 0.5 m^2/s for 1 s of tip: the filter now admits to a position uncertainty of ~0.7 m
        assertTrue(sigma(wasTipped, 0) > 0.6, "sigma " + sigma(wasTipped, 0));
        assertTrue(sigma(level, 0) < 0.1, "sigma " + sigma(level, 0));

        // The same modest vision frame (0.3 m off, std 0.1) after both
        Pose2d off = new Pose2d(0.3, 0, Rotation2d.kZero);
        level.addVisionObservation(vision(16 * TIP_STEP, off, 0.1, 0.02));
        wasTipped.addVisionObservation(vision(16 * TIP_STEP, off, 0.1, 0.02));

        assertTrue(pull(wasTipped) > 0.25, "pulled to " + pull(wasTipped));
        assertTrue(pull(level) < 0.1, "a confident filter holds its ground, got " + pull(level));
    }

    @Test
    void invalidTippedSettingsAreRejected() {
        assertThrows(
                IllegalArgumentException.class,
                () -> Config.DEFAULT.toBuilder().tippedLinearVariancePerSecond(-1.0).build());
    }

    // ---- velocity at a time ----

    /** Adds {@code count} odometry steps of 10 ms starting after {@code startSeconds}. */
    private static double drive(
            EkfPoseEstimator estimator, double startSeconds, int count, Twist2d perStep) {
        double t = startSeconds;
        for (int i = 0; i < count; i++) {
            t += 0.01;
            estimator.applyOdometryTwist(perStep, t);
        }
        return t;
    }

    @Test
    void velocityAtATimeIsTheSpeedThenNotTheSpeedNow() {
        EkfPoseEstimator estimator = new EkfPoseEstimator(KINEMATICS);
        estimator.resetPose(new Pose2d(0, 0, Rotation2d.kZero));
        estimator.applyOdometryTwist(new Twist2d(), 0.0);
        // 1 m/s for the first second, then 3 m/s for the next
        double t = drive(estimator, 0.0, 100, new Twist2d(0.01, 0.0, 0.0));
        drive(estimator, t, 100, new Twist2d(0.03, 0.0, 0.0));

        assertEquals(1.0, estimator.getVelocityAtTime(0.5).get().vxMetersPerSecond, 1e-9);
        assertEquals(3.0, estimator.getVelocityAtTime(1.5).get().vxMetersPerSecond, 1e-9);
    }

    @Test
    void velocityAtATimeIncludesTurnRateAndSidewaysMotion() {
        EkfPoseEstimator estimator = new EkfPoseEstimator(KINEMATICS);
        estimator.resetPose(new Pose2d(0, 0, Rotation2d.kZero));
        estimator.applyOdometryTwist(new Twist2d(), 0.0);
        // 2 m/s forward, 1 m/s left, turning at 3 rad/s
        drive(estimator, 0.0, 100, new Twist2d(0.02, 0.01, 0.03));

        ChassisSpeeds velocity = estimator.getVelocityAtTime(0.5).get();

        assertEquals(2.0, velocity.vxMetersPerSecond, 1e-9);
        assertEquals(1.0, velocity.vyMetersPerSecond, 1e-9);
        assertEquals(3.0, velocity.omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void velocityIsAveragedOverAShortWindowNotReadFromOneNoisyStep() {
        EkfPoseEstimator estimator = new EkfPoseEstimator(KINEMATICS);
        estimator.resetPose(new Pose2d(0, 0, Rotation2d.kZero));
        estimator.applyOdometryTwist(new Twist2d(), 0.0);
        // Steps alternate 0.5 and 1.5 m/s, so any single step is far from the true 1 m/s
        double t = 0.0;
        for (int i = 0; i < 100; i++) {
            t += 0.01;
            estimator.applyOdometryTwist(new Twist2d(i % 2 == 0 ? 0.005 : 0.015, 0.0, 0.0), t);
        }

        // A single step is 0.5 m/s off. An odd number of steps in the window cannot cancel the
        // alternation exactly, so allow 0.15
        assertEquals(1.0, estimator.getVelocityAtTime(0.5).get().vxMetersPerSecond, 0.15);
    }

    @Test
    void velocityIsUnknownWhereTheHistoryDoesNotReach() {
        EkfPoseEstimator estimator = new EkfPoseEstimator(KINEMATICS);
        estimator.resetPose(new Pose2d(0, 0, Rotation2d.kZero));
        estimator.applyOdometryTwist(new Twist2d(), 10.0);
        drive(estimator, 10.0, 10, new Twist2d(0.01, 0.0, 0.0));

        // Nothing recorded before 10 s, and nothing was ever recorded for a fresh estimator
        assertTrue(estimator.getVelocityAtTime(5.0).isEmpty());
        assertTrue(new EkfPoseEstimator(KINEMATICS).getVelocityAtTime(0.0).isEmpty());
    }

    @Test
    void aTimeAfterTheLatestStepUsesTheLatestVelocity() {
        EkfPoseEstimator estimator = new EkfPoseEstimator(KINEMATICS);
        estimator.resetPose(new Pose2d(0, 0, Rotation2d.kZero));
        estimator.applyOdometryTwist(new Twist2d(), 0.0);
        drive(estimator, 0.0, 50, new Twist2d(0.02, 0.0, 0.0));

        assertEquals(2.0, estimator.getVelocityAtTime(5.0).get().vxMetersPerSecond, 1e-9);
    }

    @Test
    void resetPoseSetsPoseAndUncertainty() {
        EkfPoseEstimator estimator = new EkfPoseEstimator(KINEMATICS);
        drive(estimator, 0.0, 3.0);

        Pose2d reset = new Pose2d(4, 2, Rotation2d.fromDegrees(30));
        estimator.resetPose(reset);

        assertEquals(reset, estimator.estimatedPose());
        assertEquals(Config.DEFAULT.resetLinearStdDev(), sigma(estimator, 0), 1e-9);
        assertEquals(Config.DEFAULT.resetAngularStdDev(), sigma(estimator, 2), 1e-9);
    }

    @Test
    void invalidConfigIsRejected() {
        assertThrows(
                IllegalArgumentException.class,
                () -> Config.DEFAULT.toBuilder().linearVariancePerMeter(-1.0).build());
        assertThrows(
                IllegalArgumentException.class,
                () -> Config.DEFAULT.toBuilder().gateRecoveryCount(0).build());
        assertThrows(
                IllegalArgumentException.class,
                () -> Config.DEFAULT.toBuilder().visionStdDevScale(Double.NaN).build());
        assertThrows(
                IllegalArgumentException.class,
                () -> Config.DEFAULT.toBuilder().gateRecoveryWindowSeconds(0.0).build());
        assertThrows(
                IllegalArgumentException.class,
                () -> Config.DEFAULT.toBuilder().gateRecoveryFraction(0.0).build());
        assertThrows(
                IllegalArgumentException.class,
                () -> Config.DEFAULT.toBuilder().gateRecoveryFraction(1.5).build());
    }
}
