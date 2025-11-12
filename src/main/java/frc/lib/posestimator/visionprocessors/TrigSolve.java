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

package frc.lib.posestimator.visionprocessors;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.io.vision.VisionIO.CameraProperties;
import frc.lib.io.vision.VisionIO.TagObservation;
import frc.lib.io.vision.VisionIO.VisionObservation;
import frc.lib.posestimator.PoseEstimator.VisionProcessor;
import frc.lib.util.GeomUtil;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import lombok.experimental.Accessors;

/**
 * A {@link VisionProcessor} implementation that estimates the robot's pose by solving trigonometric
 * relationships between a camera, a known AprilTag field layout, and a single observed tag.
 *
 * <p>
 * This class specifically targets one AprilTag at a time, to be used for more accurate alignment at
 * close distances. This is typically best used by interpolating with a {@link VisionProcessor} that
 * is more accurate at far distances, such as {@link ConstrainedSolvePnp}.
 */
@RequiredArgsConstructor
@Accessors(fluent = true)
public class TrigSolve implements VisionProcessor {

    /** Default scaling factor for linear standard deviation (distance-based uncertainty). */
    private static final double DEFAULT_LINEAR_STDDEV_FACTOR = 0.4;

    /** The field layout containing AprilTag locations. */
    private final AprilTagFieldLayout fieldLayout;

    /** Scaling factor for linear standard deviation (distance-based uncertainty). */
    @Getter
    @Setter
    private double linearStdDevFactor = DEFAULT_LINEAR_STDDEV_FACTOR;

    /**
     * The AprilTag ID that {@link #processVisionObservation(VisionObservation, Rotation2d)} uses
     * for pose estimation.
     */
    @Getter
    @Setter
    private int followedAprilTag = 0;

    /**
     * Computes a robot pose from a single AprilTag observation using trigonometric relationships.
     *
     * @param camera The camera model containing intrinsic and extrinsic parameters.
     * @param observation The detected AprilTag observation.
     * @param heading The robot’s current field-relative heading.
     * @return An {@link Optional} containing the estimated robot pose, or empty if invalid.
     */
    private Optional<Pose2d> solveTrigPosition(
        CameraProperties camera,
        TagObservation observation,
        Rotation2d heading) {

        // Convert camera extrinsics to 2D pose for transform use
        Pose2d cameraPose2d = GeomUtil.toPose3d(camera.robotToCamera()).toPose2d();

        // Compute the field-relative vector from the camera to the observed tag
        Translation2d camToTagTranslation =
            new Translation3d(
                observation.distance().in(Meters),
                new Rotation3d(
                    0,
                    -observation.pitch().in(Radians),
                    -observation.yaw().in(Radians)))
                        .rotateBy(camera.robotToCamera().getRotation())
                        .toTranslation2d()
                        .rotateBy(heading);

        // Retrieve the tag’s known field pose
        Optional<Pose2d> tagPose2d = fieldLayout.getTagPose(observation.id()).map(Pose3d::toPose2d);
        if (tagPose2d.isEmpty())
            return Optional.empty();

        // Compute where the camera must be on the field
        Translation2d fieldToCameraTranslation =
            tagPose2d.get().getTranslation().plus(camToTagTranslation.unaryMinus());

        // Combine translation and heading to determine the camera’s field-relative pose
        Pose2d cameraPoseField =
            new Pose2d(fieldToCameraTranslation, heading.plus(cameraPose2d.getRotation()));

        // Transform camera pose into robot pose
        Pose2d robotPose = cameraPoseField.transformBy(new Transform2d(cameraPose2d, Pose2d.kZero));

        // Replace rotation with odometry heading to minimize drift
        robotPose = new Pose2d(robotPose.getTranslation(), heading);

        return Optional.of(robotPose);
    }

    /**
     * Processes a {@link VisionObservation} and produces an estimated robot pose based on a single
     * AprilTag observation corresponding to the configured {@link #followedAprilTag()}.
     *
     * @param observation The vision observation containing detected tags and camera info.
     * @param heading The robot’s current field-relative heading.
     * @return An {@link Optional} {@link PoseRecord} with the estimated field-relative robot pose,
     *         or empty if the tag was not found or computation failed.
     */
    @Override
    public Optional<PoseRecord> processVisionObservation(
        VisionObservation observation,
        Rotation2d heading) {

        var tagObservations = observation.tagObservations();

        // Nothing to go off of
        if (tagObservations.isEmpty()) {
            return Optional.empty();
        }

        // The observation that matches the tag ID we're looking for
        var optionalWantedObservation = tagObservations.stream()
            .filter(obs -> obs.id() == followedAprilTag)
            .findFirst();

        // It wasn't found
        if (optionalWantedObservation.isEmpty()) {
            return Optional.empty();
        }

        TagObservation wantedObservation = optionalWantedObservation.get();

        double stdDevFactor = Math.pow(wantedObservation.distance().in(Meters), 2)
            * observation.camera().stdDevFactor();
        double linearStdDev = linearStdDevFactor * stdDevFactor;

        // This processor assumes supplied heading is perfect
        double angularStdDev = Double.POSITIVE_INFINITY;

        var a = solveTrigPosition(observation.camera(), wantedObservation, heading)
            .map(p -> new PoseRecord(new Pose3d(p), linearStdDev, angularStdDev));
        a.ifPresent(b -> Logger.recordOutput("test", b.pose()));
        return a;
    }
}
