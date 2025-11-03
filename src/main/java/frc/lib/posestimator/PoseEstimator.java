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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
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

/**
 * PoseEstimator combines swerve odometry and vision-based pose estimation (PhotonVision and
 * trig-based triangulation) to maintain an accurate estimate of the robot's pose on the field.
 */
public class PoseEstimator {

    /**
     * Represents a single odometry observation at a point in time. Includes swerve module positions
     * and optionally a gyro angle.
     */
    public static final record OdometryObservation(
        Time timestamp,
        List<SwerveModulePosition> swervePositions,
        Optional<Rotation2d> gyroAngle) {
    }

    /**
     * Stores a pose estimate derived from trig-based vision, including distance to target and
     * timestamp.
     */
    private static final record TrigPoseRecord(Pose2d pose, Distance distance, Time timestamp) {
    }

    private final AprilTagFieldLayout fieldLayout;
    private final PhotonPoseEstimator constrainedPoseEstimator;
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

    @AutoLogOutput(key = "Odometry/OdometryPose")
    private Pose2d odometryPose = Pose2d.kZero;

    @Getter
    private Pose2d estimatedPose = Pose2d.kZero;

    /**
     * Constructs a PoseEstimator with the given field layout, swerve kinematics, and heading buffer
     * size.
     *
     * @param fieldLayout AprilTag field layout used for vision-based estimation
     * @param kinematics SwerveDriveKinematics instance for odometry calculations
     * @param headingBufferSize Duration to store past headings for interpolation
     */
    public PoseEstimator(AprilTagFieldLayout fieldLayout, SwerveDriveKinematics kinematics,
        Time headingBufferSize)
    {
        this.fieldLayout = fieldLayout;
        this.kinematics = kinematics;

        constrainedPoseEstimator =
            new PhotonPoseEstimator(fieldLayout, PoseStrategy.CONSTRAINED_SOLVEPNP,
                Transform3d.kZero);

        swerveEstimator = new SwerveDrivePoseEstimator(
            kinematics, Rotation2d.kZero, lastModulePositions, Pose2d.kZero);

        odometryBuffer = TimeInterpolatableBuffer.createBuffer(headingBufferSize.in(Seconds));
    }

    /**
     * Updates the timestamp of the most recent odometry observation.
     *
     * @param timestamp Time of the latest odometry
     */
    private void updateLatestOdometryTimestamp(Time timestamp)
    {
        if (latestOdometryTimestamp.isEmpty()) {
            latestOdometryTimestamp = Optional.of(timestamp);
            return;
        }

        if (timestamp.gt(latestOdometryTimestamp.get())) {
            latestOdometryTimestamp = Optional.of(timestamp);
        }
    }

    /**
     * Adds a new odometry observation and updates the pose estimate accordingly.
     *
     * @param observation Odometry data including module positions and gyro angle
     */
    public void addOdometryObservation(OdometryObservation observation)
    {
        updateLatestOdometryTimestamp(observation.timestamp());

        double timestampSeconds = observation.timestamp().in(Seconds);
        SwerveModulePosition[] currentPositions =
            observation.swervePositions().toArray(new SwerveModulePosition[0]);

        // Calculate incremental motion using swerve kinematics
        Twist2d twist = kinematics.toTwist2d(lastModulePositions, currentPositions);
        lastModulePositions = currentPositions;

        // Update odometry pose with incremental motion
        odometryPose = odometryPose.exp(twist);

        observation.gyroAngle.ifPresent(
            angle -> odometryPose = new Pose2d(odometryPose.getTranslation(), angle));

        odometryBuffer.addSample(timestampSeconds, odometryPose);
        swerveEstimator.updateWithTime(timestampSeconds, odometryPose.getRotation(),
            currentPositions);

        estimatedPose = swerveEstimator.getEstimatedPosition();
    }

    private Optional<Rotation2d> getHeadingAtTime(Time time)
    {
        Optional<Rotation2d> gyroRelativeCurrentHeading =
            odometryBuffer.getSample(Timer.getTimestamp()).map(Pose2d::getRotation);
        if (gyroRelativeCurrentHeading.isEmpty())
            return Optional.empty();

        Optional<Rotation2d> gyroRelativeHeadingAtTime =
            odometryBuffer.getSample(time.in(Seconds)).map(Pose2d::getRotation);
        if (gyroRelativeHeadingAtTime.isEmpty())
            return Optional.empty();

        Rotation2d delta = gyroRelativeCurrentHeading.get().minus(gyroRelativeHeadingAtTime.get());
        return Optional.of(estimatedPose.getRotation().plus(delta));
    }

    private Optional<Pose2d> solveTrigPosition(Camera camera, Time timestamp,
        TagObservation observation)
    {
        Optional<Rotation2d> fieldRelativeRobotHeading = getHeadingAtTime(timestamp);
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

        Optional<Pose2d> tagPose2d =
            fieldLayout.getTagPose(observation.id()).map(Pose3d::toPose2d);
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

    /**
     * Integrates a complete vision observation containing multiple tags into the pose estimator.
     *
     * @param observation VisionObservation from PhotonVision or custom vision system
     */
    public void addVisionObservation(VisionObservation observation)
    {
        observation.tagObservations()
            .forEach(tagObservation -> add2DVisionObservation(
                observation.camera(), observation.timestamp(), tagObservation));

        int tagCount = observation.tagObservations().size();
        if (tagCount == 0 || (tagCount == 1 && observation.ambiguity() > 0.3)) {
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
        constrainedPoseEstimator.addHeadingData(timestampSeconds, heading);
        constrainedPoseEstimator.setRobotToCameraTransform(observation.camera().robotToCamera());

        Optional<EstimatedRobotPose> optionalEstimate =
            constrainedPoseEstimator.update(
                observation.photonResult(),
                Optional.of(observation.camera().cameraMatrix()),
                Optional.of(observation.camera().distCoeffs()),
                Optional.of(new ConstrainedSolvepnpParams(false, 10.0)));

        if (optionalEstimate.isEmpty()) {
            estimatedPose = swerveEstimator.getEstimatedPosition();
            return;
        }

        Pose3d estimate = optionalEstimate.get().estimatedPose;
        if (Math.abs(estimate.getZ()) > 0.75
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
        double linearStdDev = 0.4 * stdDevFactor;
        double angularStdDev = 0.4 * stdDevFactor;

        swerveEstimator.addVisionMeasurement(
            optionalEstimate.get().estimatedPose.toPose2d(),
            timestampSeconds,
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));

        estimatedPose = swerveEstimator.getEstimatedPosition();
    }

    private boolean isTrigStale(Time timestamp)
    {
        Time latestTime = latestOdometryTimestamp.orElse(Seconds.of(Timer.getTimestamp()));
        return latestTime.minus(timestamp).gte(Seconds.of(0.2));
    }

    /**
     * Returns a trig-based pose estimate for a given tag ID if it is recent and available.
     *
     * @param tagId ID of the observed tag
     * @return Optional Pose2d of the robot
     */
    public Optional<Pose2d> getTrigPose(int tagId)
    {
        Optional<TrigPoseRecord> data = Optional.ofNullable(trigPoses.get(tagId));
        if (data.isEmpty() || isTrigStale(data.get().timestamp()))
            return Optional.empty();

        return odometryBuffer.getSample(data.get().timestamp().in(Seconds))
            .map(pose2d -> data.get().pose().plus(new Transform2d(pose2d, odometryPose)));
    }
}
