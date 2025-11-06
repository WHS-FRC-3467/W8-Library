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

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.estimation.TargetModel;
import org.photonvision.estimation.VisionEstimation;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry3d;
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

        swerveEstimator = new SwerveDrivePoseEstimator(
            kinematics, Rotation2d.kZero, lastModulePositions, Pose2d.kZero);

        odometryBuffer = TimeInterpolatableBuffer.createBuffer(headingBufferSize.in(Seconds));
    }

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

    private Optional<Pose2d> solveTrigPosition(Camera camera, Time timestamp,
        TagObservation observation)
    {
        Optional<Rotation2d> fieldRelativeRobotHeading =
            swerveEstimator.sampleAt(timestamp.in(Seconds)).map(Pose2d::getRotation); // latency
                                                                                      // correction?
                                                                                      // may need IO
                                                                                      // layer
                                                                                      // change
        if (fieldRelativeRobotHeading.isEmpty())
            return Optional.empty();

        // Check if tag pose is known
        Optional<Pose2d> tagPose2d =
            fieldLayout.getTagPose(observation.id()).map(Pose3d::toPose2d);
        if (tagPose2d.isEmpty())
            return Optional.empty();

        // Calculate robot pose using trig-based triangulation
        // PV tag observations
        double camToTagNorm = observation.distance().in(Meters);
        double pitch = observation.pitch().in(Radians);
        double yaw = observation.yaw().in(Radians);

        // Spherical to Cartesian coordinate conversion (R, phi, theta) -> (x, y, z) of cam-tag norm
        // in camera frame
        Translation3d camToTagCamFrame =
            new Translation3d(camToTagNorm, new Rotation3d(0, -pitch, -yaw));
        // Rotate to robot frame
        Translation3d camToTagRobotFrame =
            camToTagCamFrame.rotateBy(camera.robotToCamera().getRotation());
        // Compute robot position in field frame
        Translation3d robotToTagRobotFrame =
            camera.robotToCamera().getTranslation().plus(camToTagRobotFrame);
        Translation2d fieldToRobot =
            tagPose2d.get().getTranslation().minus(robotToTagRobotFrame.toTranslation2d());

        // Compute robot heading using both odometry and observed yaw
        // Tag yaw gives robot heading relative to the tag
        Rotation2d observedHeading = tagPose2d.get().getRotation().minus(new Rotation2d(yaw));
        // Fuse with odometry (weighting can be tuned -- weightVision parameter based on angular
        // velocity or Kalman filter).
        Rotation2d fusedHeading =
            observedHeading.interpolate(fieldRelativeRobotHeading.get(), 0.05);

        // Build final robot pose
        Pose2d robotPose = new Pose2d(fieldToRobot, fusedHeading);

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
                10.0).map(estimate -> GeomUtil.toPose3d(estimate.best));

        if (optionalEstimate.isEmpty()) {
            estimatedPose = swerveEstimator.getEstimatedPosition();
            return;
        }

        Pose3d estimate = optionalEstimate.get();
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
            optionalEstimate.get().toPose2d(),
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

}
