/*
 * Copyright (C) 2025 Windham Windup
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
import java.util.List;
import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Time;
import lombok.Getter;

/**
 * Integrates swerve-drive odometry observations to maintain a continuous estimate of the robot's
 * field-relative pose over time. This is essentially a {@link SwerveDriveOdometry} with more
 * outputs to be utilized in a {@link PoseEstimator}.
 *
 * <p>
 * This class tracks the robot's module positions, integrates motion deltas into a {@link Pose2d}
 * estimate, and maintains a time-buffered history of poses for interpolation. It is intended to be
 * used by a higher-level {@link PoseEstimator} that fuses odometry with vision or other sensors. Do
 * not use this class on it's own, use {@link SwerveDriveOdometry}.
 *
 * <p>
 * Responsibilities:
 * <ul>
 * <li>Track last known module positions for twist calculation.</li>
 * <li>Compute incremental robot motion and update the odometry pose.</li>
 * <li>Maintain a timestamped buffer of recent poses for temporal interpolation.</li>
 * </ul>
 */
public class SwerveOdometer {
    /**
     * Represents a single odometry observation from the swerve drive.
     *
     * <p>
     * This record encapsulates the information required to update the robot's odometry:
     * <ul>
     * <li>{@code timestamp} — the time at which the observation was made</li>
     * <li>{@code swervePositions} — the positions of all swerve modules at this time</li>
     * <li>{@code gyroAngle} — an optional gyro heading to correct orientation drift</li>
     * </ul>
     *
     * <p>
     * Instances of this record are intended to be passed to
     * {@link SwerveOdometer#addOdometryObservation(OdometryObservation)} or
     * {@link PoseEstimator#addOdometryObservation(OdometryObservation)} to update the odometry
     * estimate and store the pose in the time-buffer for interpolation with vision or other
     * sensors.
     *
     * @param timestamp the time of the observation
     * @param swervePositions the current positions of the swerve modules
     * @param gyroAngle optional gyro angle to correct heading drift
     */
    public static final record OdometryObservation(
        Time timestamp,
        List<SwerveModulePosition> swervePositions,
        Optional<Rotation2d> gyroAngle) {
    }

    /** The swerve drive kinematics used to compute motion deltas. */
    private final SwerveDriveKinematics kinematics;

    /** Tracks last known module positions for computing motion deltas. */
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };

    /** Stores recent odometry poses for interpolation purposes. */
    @Getter
    private final TimeInterpolatableBuffer<Pose2d> odometryBuffer;

    /** The timestamp of the most recent odometry observation. */
    @Getter
    private Optional<Time> latestOdometryTimestamp = Optional.empty();

    /** The most recent odometry-based robot pose. */
    @Getter
    private Pose2d odometryPose = Pose2d.kZero;

    /**
     * Constructs a new {@code OdometryPoseIntegrator}.
     *
     * @param kinematics the swerve drive kinematics for computing motion deltas
     * @param headingBufferSize the duration for which odometry poses are buffered for interpolation
     */
    public SwerveOdometer(SwerveDriveKinematics kinematics, Time headingBufferSize)
    {
        this.kinematics = kinematics;
        odometryBuffer = TimeInterpolatableBuffer.createBuffer(headingBufferSize.in(Seconds));
    }

    /**
     * Updates the stored timestamp if the provided timestamp is newer than the current one.
     *
     * @param timestamp the new odometry timestamp to consider
     */
    private void updateLatestOdometryTimestamp(Time timestamp)
    {
        if (latestOdometryTimestamp.isEmpty() || timestamp.gt(latestOdometryTimestamp.get())) {
            latestOdometryTimestamp = Optional.of(timestamp);
        }
    }

    /**
     * Adds an odometry observation to the integrator.
     *
     * <p>
     * This method performs the following steps:
     * <ul>
     * <li>Updates the latest odometry timestamp.</li>
     * <li>Computes the robot's incremental motion (twist) from the last known module
     * positions.</li>
     * <li>Integrates the twist into the current odometry pose.</li>
     * <li>If a gyro angle is provided, corrects the pose heading accordingly.</li>
     * <li>Stores the resulting pose in the time-buffer for interpolation.</li>
     * </ul>
     *
     * @param observation an {@link OdometryObservation} containing module positions, timestamp, and
     *        an optional gyro angle
     */
    public void addOdometryObservation(OdometryObservation observation)
    {
        // Update latest timestamp if newer
        updateLatestOdometryTimestamp(observation.timestamp());

        double timestampSeconds = observation.timestamp().in(Seconds);
        SwerveModulePosition[] currentPositions =
            observation.swervePositions().toArray(new SwerveModulePosition[0]);

        // Compute robot motion (twist) since last update
        Twist2d twist = kinematics.toTwist2d(lastModulePositions, currentPositions);
        lastModulePositions = currentPositions;

        // Integrate twist into odometry pose
        odometryPose = odometryPose.exp(twist);

        // If gyro angle is available, correct heading drift
        observation.gyroAngle().ifPresent(
            angle -> odometryPose = new Pose2d(odometryPose.getTranslation(), angle));

        // Store pose for later interpolation (e.g., vision sync)
        odometryBuffer.addSample(timestampSeconds, odometryPose);
    }
}
