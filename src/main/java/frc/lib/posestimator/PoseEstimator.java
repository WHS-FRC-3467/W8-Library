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

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;

import frc.lib.io.vision.VisionIO.VisionObservation;
import frc.lib.posestimator.PoseEstimator.VisionProcessor.PoseRecord;
import frc.lib.posestimator.SwerveOdometer.OdometryObservation;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

/**
 * Estimates the robot's field-relative pose using a combination of swerve-drive odometry and
 * AprilTag-based vision observations.
 *
 * <p>
 * This class fuses kinematic odometry data with visual pose estimates to produce a more accurate
 * and robust {@link Pose2d} estimate of the robot's position and orientation on the field. It
 * supports single- and multi-tag vision measurements, distance-based confidence scaling, and
 * single-tag based triangulation for zero-ambiguity pose estimation with close distances.
 *
 * <p>
 * Pose updates occur via two main data sources:
 * <ul>
 * <li>{@link #addOdometryObservation(OdometryObservation)} — incremental odometry updates</li>
 * <li>{@link #addVisionObservation(VisionObservation)} — vision-based pose corrections</li>
 * </ul>
 *
 * <p>
 * Recent odometry samples are also buffered to support timestamp interpolation.
 */
@Accessors(fluent = true)
public class PoseEstimator {
    public static interface VisionProcessor {
        /** Stores a vision pose estimate along with computed uncertainty metrics. */
        public static final record PoseRecord(
            Pose3d pose,
            double linearStdDev,
            double angularStdDev) {
        }

        Optional<PoseRecord> processVisionObservation(
            VisionObservation observation,
            Rotation2d heading);
    }

    private static final double DEFAULT_ODOMETRY_BUFFER_SIZE_SECONDS = 2;
    private static final double DEFAULT_ODOMETRY_LINEAR_STDDEV = 0.01;
    private static final double DEFAULT_ODOMETRY_ANGULAR_STDDEV = 0.01;

    private final SwerveOdometer odometer;
    private final VisionProcessor visionProcessor;

    /**
     * Sets the linear standard deviation (noise) of odometry
     *
     * @param linearOdometryStdDev The linear standard deviation
     */
    @Setter
    private double linearOdometryStdDev = DEFAULT_ODOMETRY_LINEAR_STDDEV;

    /**
     * Sets the angular standard deviation (noise) of odometry
     *
     * @param angularOdometryStdDev The angular standard deviation
     */
    @Setter
    private double angularOdometryStdDev = DEFAULT_ODOMETRY_ANGULAR_STDDEV;

    /**
     * Returns the current fused estimated pose of the robot.
     *
     * <p>
     * This pose includes odometry integration and accepted vision corrections.
     */
    @Getter
    private Pose2d estimatedPose = Pose2d.kZero;

    /**
     * Constructs a new {@code PoseEstimator}.
     * 
     * @param visionProcessor The {@code VisionProcessor} to use
     * @param kinematics The robot's swerve drive kinematics model
     * @param odometryBufferSize The maximum duration of stored odometry samples used for
     *        interpolation
     */
    public PoseEstimator(
        VisionProcessor visionProcessor,
        SwerveDriveKinematics kinematics,
        Time odometryBufferSize) {
        this.visionProcessor = visionProcessor;
        odometer = new SwerveOdometer(kinematics, odometryBufferSize);
    }

    /**
     * Constructs a new {@code PoseEstimator} using a default heading buffer size.
     *
     * @param visionProcessor The {@code VisionProcessor} to use
     * @param kinematics the robot's swerve drive kinematics model
     */
    public PoseEstimator(VisionProcessor visionProcessor, SwerveDriveKinematics kinematics) {
        this(visionProcessor, kinematics, Seconds.of(DEFAULT_ODOMETRY_BUFFER_SIZE_SECONDS));
    }

    /**
     * Adds an odometry observation to the estimator.
     *
     * <p>
     * This updates the internal swerve odometry, tracks the pose over time, and updates the fused
     * pose estimate.
     *
     * @param observation the odometry observation containing timestamp, swerve module states, and
     *        an optional gyro heading
     */
    public void addOdometryObservation(OdometryObservation observation) {
        Pose2d lastOdometryPose = odometer.getOdometryPose();
        odometer.addOdometryObservation(observation);
        Pose2d newOdometryPose = odometer.getOdometryPose();

        Twist2d twist = lastOdometryPose.log(newOdometryPose);

        estimatedPose = estimatedPose.exp(twist);
    }

    /**
     * Attempts to get the tranform between the pose of the robot at the specified timestamp and
     * now.
     * 
     * <p>
     * This method will fail if the odometer's buffer does not contain enough measurements to
     * interpolate.
     * 
     * @param time The timestamp to get the transform from
     * @return The optional transform from {@code time} to now
     */
    private Optional<Transform2d> getPoseDelta(Time time) {
        var optionalOdometryPoseAtTime = odometer.getOdometryBuffer().getSample(time.in(Seconds));
        if (optionalOdometryPoseAtTime.isEmpty()) {
            return Optional.empty();
        }
        Pose2d odometryPoseAtTime = optionalOdometryPoseAtTime.get();

        Transform2d thenToNow = odometryPoseAtTime.minus(odometer.getOdometryPose());

        return Optional.of(thenToNow);
    }

    /**
     * Adds a vision observation to the estimator.
     *
     * <p>
     * Processes detected AprilTags to correct the estimated robot pose. Depending on the number of
     * tags and observation ambiguity, the estimator may accept or reject the vision update. The
     * update uncertainty scales with distance and tag count.
     *
     * @param observation the vision observation containing tag detections, timestamp, and camera
     *        metadata
     */
    public void addVisionObservation(VisionObservation observation) {
        // Attempt to get heading. Fails if the odometer has not recorded
        // a measurement near this timestamp
        var optionalPoseDelta = getPoseDelta(observation.timestamp());
        if (optionalPoseDelta.isEmpty()) {
            return;
        }
        Transform2d poseDelta = optionalPoseDelta.get();

        Pose2d oldPose = estimatedPose.plus(poseDelta.inverse());
        var optionalGlobalPoseRecord =
            visionProcessor.processVisionObservation(observation, oldPose.getRotation());
        if (optionalGlobalPoseRecord.isEmpty()) {
            return;
        }
        PoseRecord newVisionPose = optionalGlobalPoseRecord.get();

        // Solve Kalman gain matrix given observation standard deviations
        // Copied from:
        // https://github.com/wpilibsuite/allwpilib/blob/b8d6bc2eb1b6cea10d1179939114d041945e172a/wpimath/src/main/java/edu/wpi/first/math/estimator/PoseEstimator.java#L93-L109
        double[] visionStdDevs = {
                newVisionPose.linearStdDev(), // X axis
                newVisionPose.linearStdDev(), // Y axis
                newVisionPose.angularStdDev()}; // Rotation

        double[] odometryStdDevs = {
                linearOdometryStdDev, // X axis
                linearOdometryStdDev, // Y axis
                angularOdometryStdDev // Rotation
        };

        Matrix<N3, N3> visionKalmanGain = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; ++row) {
            double odometryStdDev = odometryStdDevs[row];
            if (odometryStdDev == 0.0) {
                visionKalmanGain.set(row, row, 0.0);
            } else {
                visionKalmanGain.set(row, row, odometryStdDev
                    / (odometryStdDev + Math.sqrt(odometryStdDev * visionStdDevs[row])));
            }
        }

        // Transform between our best estimated pose at the time the frame was captured to where the
        // camera is saying we should be, unscaled (without any Kalman gain applied)
        // https://github.com/wpilibsuite/allwpilib/blob/b8d6bc2eb1b6cea10d1179939114d041945e172a/wpimath/src/main/java/edu/wpi/first/math/estimator/PoseEstimator.java#L276-L292
        Transform2d unscaledVisionCorrection =
            new Transform2d(oldPose, newVisionPose.pose().toPose2d());

        // Scale the vision correction by the Kalman gain
        var scaledVisionCorrectionVector = visionKalmanGain.times(
            VecBuilder.fill(
                unscaledVisionCorrection.getX(),
                unscaledVisionCorrection.getY(),
                unscaledVisionCorrection.getRotation().getRadians()));

        // Convert to Transform2d
        Transform2d scaledVisionCorrection = new Transform2d(
            scaledVisionCorrectionVector.get(0, 0),
            scaledVisionCorrectionVector.get(1, 0),
            Rotation2d.fromRadians(scaledVisionCorrectionVector.get(2, 0)));

        estimatedPose = oldPose
            .transformBy(scaledVisionCorrection) // Adjust by the correction
            .transformBy(poseDelta); // Bring back to present time (latency comp)
    }

    /**
     * Resets the pose estimator to a known field-relative pose.
     *
     * <p>
     * This method should be called when the robot's absolute position on the field is known — for
     * example, at the start of autonomous or after a vision-based correction. It reinitializes both
     * the odometry integrator and the estimator’s internal pose state to the specified pose.
     *
     * @param pose the known {@link Pose2d} representing the robot’s field-relative position
     */
    public void resetPose(Pose2d pose) {
        odometer.resetPose(pose);
        estimatedPose = pose;
    }
}
