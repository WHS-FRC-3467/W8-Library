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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Time;

import lombok.Getter;
import lombok.experimental.Accessors;

import java.util.Optional;

/**
 * Integrates swerve-drive odometry observations to maintain a continuous estimate of the robot's
 * field-relative pose over time. This is essentially a {@link SwerveDriveOdometry} with more
 * outputs to be utilized in a {@link PoseEstimator}.
 *
 * <p>This class tracks the robot's module positions, integrates motion deltas into a {@link Pose2d}
 * estimate, and maintains a time-buffered history of poses for interpolation. It is intended to be
 * used by a higher-level {@link PoseEstimator} that fuses odometry with vision or other sensors. Do
 * not use this class on it's own, use {@link SwerveDriveOdometry}.
 */
@Accessors(fluent = true)
public class SwerveOdometry {
    /** Represents a single odometry observation from the swerve drive. */
    @SuppressWarnings("ArrayRecordComponent")
    public static final record OdometryObservation(
            Time timestamp,
            SwerveModulePosition[] swervePositions,
            Optional<Rotation2d> gyroAngle) {}

    /** The swerve drive kinematics used to compute motion deltas. */
    private final SwerveDriveKinematics kinematics;

    /** Stores recent odometry poses for interpolation purposes. */
    @Getter private final TimeInterpolatableBuffer<Pose2d> odometryBuffer;

    /** Tracks last known module positions for computing motion deltas. */
    private SwerveModulePosition[] lastModulePositions =
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };

    /** The offset from the gyro's 0 at boot and the last reset frame of rotation */
    private Rotation2d gyroOffset = Rotation2d.kZero;

    /** The most recent odometry-based robot pose. */
    @Getter private Pose2d odometryPose = new Pose2d(1.0, 1.0, gyroOffset);

    /**
     * True when a pose reset has occurred and the next odometry observation should re-seed the
     * module position baseline to avoid a spurious jump.
     */
    private boolean pendingPositionReset = false;

    /**
     * Constructs a new {@code SwerveOdometry}.
     *
     * @param kinematics the swerve drive kinematics for computing motion deltas
     * @param odometryBufferSize the duration for which odometry poses are buffered for
     *     interpolation
     */
    public SwerveOdometry(SwerveDriveKinematics kinematics, Time odometryBufferSize) {
        this.kinematics = kinematics;
        odometryBuffer = TimeInterpolatableBuffer.createBuffer(odometryBufferSize.in(Seconds));
    }

    /**
     * Adds an odometry observation to the integrator.
     *
     * @param observation an {@link OdometryObservation} containing module positions, timestamp, and
     *     an optional gyro angle
     */
    public void addOdometryObservation(OdometryObservation observation) {
        double timestampSeconds = observation.timestamp().in(Seconds);
        SwerveModulePosition[] currentPositions = observation.swervePositions();

        if (pendingPositionReset) {
            lastModulePositions = copyPositions(currentPositions);
            observation
                    .gyroAngle()
                    .ifPresent(angle -> gyroOffset = odometryPose.getRotation().minus(angle));
            odometryBuffer.addSample(timestampSeconds, odometryPose);
            pendingPositionReset = false;
            return;
        }

        Twist2d twist = kinematics.toTwist2d(lastModulePositions, currentPositions);

        // Copy positions so callers can't accidentally mutate our history.
        lastModulePositions = copyPositions(currentPositions);

        // Integrate twist into odometry pose
        Pose2d candidatePose = odometryPose.exp(twist);

        // If gyro angle is available, correct heading drift
        if (observation.gyroAngle().isPresent()) {
            Rotation2d angle = observation.gyroAngle().get();
            candidatePose = new Pose2d(candidatePose.getTranslation(), angle.plus(gyroOffset));
        }

        odometryPose = candidatePose;

        // Store pose for later interpolation (e.g., vision sync)
        odometryBuffer.addSample(timestampSeconds, odometryPose);
    }

    /**
     * Adds an odometry observation to the integrator, ignoring everything except gyro.
     *
     * @param observation an {@link OdometryObservation} containing an optional gyro angle
     */
    public void addGyroObservation(OdometryObservation observation) {
        double timestampSeconds = observation.timestamp().in(Seconds);

        if (pendingPositionReset) {
            // While awaiting a position reset, still apply gyro heading updates to the pose
            observation
                    .gyroAngle()
                    .ifPresent(
                            angle ->
                                    odometryPose =
                                            new Pose2d(
                                                    odometryPose.getTranslation(),
                                                    angle.plus(gyroOffset)));
            odometryBuffer.addSample(timestampSeconds, odometryPose);
            return;
        }

        // If gyro angle is available, correct heading drift
        observation
                .gyroAngle()
                .ifPresent(
                        angle ->
                                odometryPose =
                                        new Pose2d(
                                                odometryPose.getTranslation(),
                                                angle.plus(gyroOffset)));

        // Store pose for later interpolation (e.g., vision sync)
        odometryBuffer.addSample(timestampSeconds, odometryPose);
    }

    private static SwerveModulePosition[] copyPositions(SwerveModulePosition[] positions) {
        SwerveModulePosition[] copy = new SwerveModulePosition[positions.length];
        System.arraycopy(positions, 0, copy, 0, positions.length);
        return copy;
    }

    /**
     * Resets the internal odometry state to a known pose.
     *
     * @param pose the new field-relative {@link Pose2d} representing the robot's known position
     */
    public void resetPose(Pose2d pose) {
        gyroOffset = pose.getRotation().minus(odometryPose.getRotation().minus(gyroOffset));
        odometryPose = pose;
        odometryBuffer.clear();
        pendingPositionReset = true;
    }
}
