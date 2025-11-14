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
import java.util.function.Predicate;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import frc.lib.devices.AprilTagCamera.CameraProperties;
import frc.lib.posestimator.SwerveOdometry.OdometryObservation;
import frc.lib.posestimator.visionprocessors.VisionProcessor;
import frc.lib.posestimator.visionprocessors.VisionProcessor.PoseRecord;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

@Accessors(fluent = true)
public class PoseEstimator {
    private static final double DEFAULT_ODOMETRY_BUFFER_SIZE_SECONDS = 2;

    private final SwerveOdometry odometer;
    private final VisionProcessor visionProcessor;

    private double linearOdometryStdDevSquared;

    private double angularOdometryStdDevSquared;

    @Setter
    private Optional<Predicate<PoseRecord>> visionPoseFilter;

    @Getter
    private Pose2d estimatedPose = Pose2d.kZero;

    public PoseEstimator(
        VisionProcessor visionProcessor,
        SwerveDriveKinematics kinematics,
        Time odometryBufferSize,
        double linearOdometryStdDev,
        double angularOdometryStdDev)
    {
        this.visionProcessor = visionProcessor;
        this.linearOdometryStdDevSquared = Math.pow(linearOdometryStdDev, 2);
        this.angularOdometryStdDevSquared = Math.pow(angularOdometryStdDev, 2);
        odometer = new SwerveOdometry(kinematics, odometryBufferSize);
    }

    public PoseEstimator(
        VisionProcessor visionProcessor,
        SwerveDriveKinematics kinematics,
        double linearOdometryStdDev,
        double angularOdometryStdDev)
    {
        this(
            visionProcessor,
            kinematics,
            Seconds.of(DEFAULT_ODOMETRY_BUFFER_SIZE_SECONDS),
            linearOdometryStdDev,
            angularOdometryStdDev);
    }

    public void addOdometryObservation(OdometryObservation observation)
    {
        Pose2d lastOdometryPose = odometer.getOdometryPose();
        odometer.addOdometryObservation(observation);
        Pose2d newOdometryPose = odometer.getOdometryPose();

        Twist2d twist = lastOdometryPose.log(newOdometryPose);

        estimatedPose = estimatedPose.exp(twist);
    }

    private Optional<Transform2d> getPoseDelta(double timestampSeconds)
    {
        var optionalOdometryPoseAtTime = odometer.getOdometryBuffer().getSample(timestampSeconds);
        if (optionalOdometryPoseAtTime.isEmpty()) {
            return Optional.empty();
        }
        Pose2d odometryPoseAtTime = optionalOdometryPoseAtTime.get();

        Transform2d thenToNow = odometryPoseAtTime.minus(odometer.getOdometryPose());

        return Optional.of(thenToNow);
    }

    public void addVisionObservation(PhotonPipelineResult result, CameraProperties camera)
    {
        // Attempt to get heading. Fails if the odometer has not recorded
        // a measurement near this timestamp
        var optionalPoseDelta = getPoseDelta(result.getTimestampSeconds());
        if (optionalPoseDelta.isEmpty()) {
            return;
        }
        Transform2d poseDelta = optionalPoseDelta.get();

        Pose2d oldPose = estimatedPose.plus(poseDelta.inverse());
        var optionalGlobalPoseRecord =
            visionProcessor.processVisionObservation(result, camera, oldPose.getRotation());
        if (optionalGlobalPoseRecord.isEmpty()) {
            return;
        }
        PoseRecord newVisionPose = optionalGlobalPoseRecord.get();

        // Test on user-defined predicate
        if (visionPoseFilter.isPresent() && !visionPoseFilter.get().test(newVisionPose)) {
            return;
        }

        // Solve Kalman gain matrix given observation standard deviations
        // Copied from:
        // https://github.com/wpilibsuite/allwpilib/blob/b8d6bc2eb1b6cea10d1179939114d041945e172a/wpimath/src/main/java/edu/wpi/first/math/estimator/PoseEstimator.java#L93-L109
        double[] visionStdDevs = {
                Math.pow(newVisionPose.linearStdDev(), 2), // X axis
                Math.pow(newVisionPose.linearStdDev(), 2), // Y axis
                Math.pow(newVisionPose.angularStdDev(), 2)}; // Rotation

        double[] odometryStdDevs = {
                linearOdometryStdDevSquared, // X axis
                linearOdometryStdDevSquared, // Y axis
                angularOdometryStdDevSquared // Rotation
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
    public void resetPose(Pose2d pose)
    {
        odometer.resetPose(pose);
        estimatedPose = pose;
    }
}
