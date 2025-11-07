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

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.photonvision.estimation.TargetModel;
import org.photonvision.estimation.VisionEstimation;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;

import frc.lib.io.vision.VisionIO.Camera;
import frc.lib.io.vision.VisionIO.TagObservation;
import frc.lib.io.vision.VisionIO.VisionObservation;
import frc.lib.util.GeomUtil;

import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

@Accessors(chain = true)
public class PoseEstimator {

    private static final double DEFAULT_AMBIGUITY_THRESHOLD = 0.3;
    private static final double DEFAULT_MAX_Z_METERS = 0.75;
    private static final double DEFAULT_LINEAR_STDDEV_FACTOR = 0.4;
    private static final double DEFAULT_ANGULAR_STDDEV_FACTOR = 0.4;
    private static final double DEFAULT_TRIG_STALE_TIME_SECONDS = 0.2;
    private static final double DEFAULT_GYRO_HEADING_SCALE_FACTOR = 10.0;

    @Setter
    private double ambiguityThreshold = DEFAULT_AMBIGUITY_THRESHOLD;
    @Setter
    private double maxZMeters = DEFAULT_MAX_Z_METERS;
    @Setter
    private double linearStdDevFactor = DEFAULT_LINEAR_STDDEV_FACTOR;
    @Setter
    private double angularStdDevFactor = DEFAULT_ANGULAR_STDDEV_FACTOR;
    @Setter
    private double trigStaleTimeSeconds = DEFAULT_TRIG_STALE_TIME_SECONDS;
    @Setter
    private double gyroHeadingScaleFactor = DEFAULT_GYRO_HEADING_SCALE_FACTOR;

    private final AprilTagFieldLayout fieldLayout;
    private final SwerveDrivePoseEstimator swerveEstimator;
    private final SwerveDriveKinematics kinematics;

    private Optional<Time> latestOdometryTimestamp = Optional.empty();
    private final TimeInterpolatableBuffer<Pose2d> odometryBuffer;
    private final Map<Integer, TrigPoseRecord> trigPoses = new HashMap<>();

    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };

    @Getter
    private Pose2d odometryPose = Pose2d.kZero;

    @Getter
    private Pose2d estimatedPose = Pose2d.kZero;

    public PoseEstimator(AprilTagFieldLayout fieldLayout, SwerveDriveKinematics kinematics,
        Time headingBufferSize)
    {
        this.fieldLayout = fieldLayout;
        this.kinematics = kinematics;

        swerveEstimator = new SwerveDrivePoseEstimator(
            kinematics, Rotation2d.kZero, lastModulePositions, Pose2d.kZero);

        odometryBuffer = TimeInterpolatableBuffer.createBuffer(headingBufferSize.in(Seconds));
    }

    private void updateLatestOdometryTimestamp(Time timestamp)
    {
        if (latestOdometryTimestamp.isEmpty() || timestamp.gt(latestOdometryTimestamp.get())) {
            latestOdometryTimestamp = Optional.of(timestamp);
        }
    }

    public void addOdometryObservation(OdometryObservation observation)
    {
        updateLatestOdometryTimestamp(observation.timestamp());

        double timestampSeconds = observation.timestamp().in(Seconds);
        SwerveModulePosition[] currentPositions =
            observation.swervePositions().toArray(new SwerveModulePosition[0]);

        Twist2d twist = kinematics.toTwist2d(lastModulePositions, currentPositions);
        lastModulePositions = currentPositions;

        odometryPose = odometryPose.exp(twist);

        observation.gyroAngle.ifPresent(
            angle -> odometryPose = new Pose2d(odometryPose.getTranslation(), angle));

        odometryBuffer.addSample(timestampSeconds, odometryPose);
        swerveEstimator.updateWithTime(timestampSeconds, odometryPose.getRotation(),
            currentPositions);

        estimatedPose = swerveEstimator.getEstimatedPosition();
    }

    private Optional<Pose2d> solveTrigPosition(Camera camera, Time timestamp,
        TagObservation observation)
    {
        Optional<Rotation2d> fieldRelativeRobotHeading =
            swerveEstimator.sampleAt(timestamp.in(Seconds)).map(Pose2d::getRotation);
        if (fieldRelativeRobotHeading.isEmpty())
            return Optional.empty();

        Pose2d cameraPose2d = GeomUtil.toPose3d(camera.robotToCamera()).toPose2d();

        Translation2d camToTagTranslation =
            new Translation3d(
                observation.distance().in(Meters),
                new Rotation3d(
                    0,
                    -observation.pitch().in(Radians),
                    -observation.yaw().in(Radians)))
                        .rotateBy(camera.robotToCamera().getRotation())
                        .toTranslation2d()
                        .rotateBy(fieldRelativeRobotHeading.get());

        Optional<Pose2d> tagPose2d = fieldLayout.getTagPose(observation.id()).map(Pose3d::toPose2d);
        if (tagPose2d.isEmpty())
            return Optional.empty();

        Translation2d fieldToCameraTranslation =
            tagPose2d.get().getTranslation().plus(camToTagTranslation.unaryMinus());

        Pose2d cameraPoseField =
            new Pose2d(fieldToCameraTranslation,
                fieldRelativeRobotHeading.get().plus(cameraPose2d.getRotation()));

        Pose2d robotPose = cameraPoseField.transformBy(
            new Transform2d(cameraPose2d, Pose2d.kZero));

        robotPose = new Pose2d(robotPose.getTranslation(), fieldRelativeRobotHeading.get());

        return Optional.of(robotPose);
    }

    private void add2DVisionObservation(Camera camera, Time timestamp,
        TagObservation observation)
    {
        Optional<Pose2d> pose = solveTrigPosition(camera, timestamp, observation);
        pose.ifPresent(p -> trigPoses.put(observation.id(),
            new TrigPoseRecord(p, observation.distance(), timestamp)));
    }

    public void addVisionObservation(VisionObservation observation)
    {
        observation.tagObservations()
            .forEach(tagObservation -> add2DVisionObservation(
                observation.camera(), observation.timestamp(), tagObservation));

        int tagCount = observation.tagObservations().size();
        if (tagCount == 0 || (tagCount == 1 && observation.ambiguity() > ambiguityThreshold)) {
            estimatedPose = swerveEstimator.getEstimatedPosition();
            return;
        }

        double timestampSeconds = observation.timestamp().in(Seconds);
        Optional<Pose2d> sampledPose = swerveEstimator.sampleAt(timestampSeconds);
        if (sampledPose.isEmpty()) {
            estimatedPose = swerveEstimator.getEstimatedPosition();
            return;
        }

        Rotation2d heading = sampledPose.get().getRotation();
        Optional<Pose3d> optionalEstimate =
            VisionEstimation.estimateRobotPoseConstrainedSolvepnp(
                observation.camera().cameraMatrix(),
                observation.camera().distCoeffs(),
                observation.tagObservations().stream().map(TagObservation::toPhotonTarget).toList(),
                observation.camera().robotToCamera(),
                GeomUtil.toPose3d(observation.bestCameraToTarget()
                    .plus(observation.camera().robotToCamera().inverse())),
                fieldLayout,
                TargetModel.kAprilTag36h11,
                false,
                heading,
                gyroHeadingScaleFactor)
                .map(estimate -> GeomUtil.toPose3d(estimate.best));

        if (optionalEstimate.isEmpty()) {
            estimatedPose = swerveEstimator.getEstimatedPosition();
            return;
        }

        Pose3d estimate = optionalEstimate.get();
        if (Math.abs(estimate.getZ()) > maxZMeters
            || estimate.getX() < 0.0
            || estimate.getX() > fieldLayout.getFieldLength()
            || estimate.getY() < 0.0
            || estimate.getY() > fieldLayout.getFieldWidth()) {
            estimatedPose = swerveEstimator.getEstimatedPosition();
            return;
        }

        double averageTagDistance = observation.tagObservations().stream()
            .mapToDouble(tag -> tag.distance().in(Meters))
            .average()
            .orElse(0.0);

        double stdDevFactor =
            Math.pow(averageTagDistance, 2.0) / observation.tagObservations().size();
        double linearStdDev = linearStdDevFactor * stdDevFactor;
        double angularStdDev = angularStdDevFactor * stdDevFactor;

        swerveEstimator.addVisionMeasurement(
            optionalEstimate.get().toPose2d(),
            timestampSeconds,
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));

        estimatedPose = swerveEstimator.getEstimatedPosition();
    }

    private boolean isTrigStale(Time timestamp)
    {
        Time latestTime = latestOdometryTimestamp.orElse(Seconds.of(Timer.getTimestamp()));
        return latestTime.minus(timestamp).gte(Seconds.of(trigStaleTimeSeconds));
    }

    public Optional<Pose2d> getTrigPose(int tagId)
    {
        if (!trigPoses.containsKey(tagId)) {
            return Optional.empty();
        }
        var data = trigPoses.get(tagId);
        if (isTrigStale(data.timestamp)) {
            return Optional.empty();
        }

        var sample = odometryBuffer.getSample(data.timestamp().in(Seconds));
        return sample.map(pose2d -> data.pose().plus(new Transform2d(pose2d, odometryPose)));
    }

    // === Records ===
    public static final record OdometryObservation(
        Time timestamp,
        List<SwerveModulePosition> swervePositions,
        Optional<Rotation2d> gyroAngle) {
    }

    private static final record TrigPoseRecord(Pose2d pose, Distance distance, Time timestamp) {
    }
}
