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
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import org.photonvision.estimation.TargetModel;
import org.photonvision.estimation.VisionEstimation;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.lib.io.vision.VisionIO.Camera;
import frc.lib.io.vision.VisionIO.TagObservation;
import frc.lib.io.vision.VisionIO.VisionObservation;
import frc.lib.util.GeomUtil;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

/**
 * Processes vision-based AprilTag observations to estimate the robot’s field-relative pose.
 *
 * <p>
 * This class handles both:
 * <ul>
 * <li><b>2D triangulation</b> — computing single-tag trigonometric estimates using measured
 * distance, yaw, and pitch.</li>
 * <li><b>3D PnP solving</b> — computing global robot pose using multi-tag constrained PnP solutions
 * via {@link VisionEstimation}.</li>
 * </ul>
 *
 * <p>
 * Each incoming {@link VisionObservation} is evaluated for validity and consistency based on tag
 * count, ambiguity, distance, and field boundaries. The resulting pose estimates are stored or
 * returned as {@link TrigPoseRecord} or {@link PNPPoseRecord}, each containing pose data and
 * uncertainty metrics for later fusion with odometry.
 *
 * <p>
 * This class does not directly modify a {@link SwerveDrivePoseEstimator} or fused pose; it serves
 * as a dedicated vision processing component for use within a higher-level estimator.
 */
@AllArgsConstructor
@Accessors(fluent = true)
public class VisionProcessor {

    /** Stores a triangulated 2D pose estimate for a single AprilTag observation. */
    public static final record TrigPoseRecord(Pose2d pose, Distance distance, Time timestamp) {
    }

    /** Stores a 3D PnP pose estimate along with computed uncertainty metrics. */
    public static final record PNPPoseRecord(Pose3d pose, double linearStdDev,
        double angularStdDev) {
    }

    private final AprilTagFieldLayout fieldLayout;

    /** Maximum acceptable ambiguity for a single-tag pose estimate. */
    @Getter
    @Setter
    private double ambiguityThreshold;

    /** Maximum acceptable height (in meters) of a pose estimate’s Z coordinate. */
    @Getter
    @Setter
    private double maxZMeters;

    /** Weighting factor applied to gyro heading in the constrained PnP solver. */
    @Getter
    @Setter
    private double gyroHeadingScaleFactor;

    /** Scaling factor for linear standard deviation (distance-based uncertainty). */
    @Getter
    @Setter
    private double linearStdDevFactor;

    /** Scaling factor for angular standard deviation (orientation-based uncertainty). */
    @Getter
    @Setter
    private double angularStdDevFactor;

    /** Tracks the most recent triangulated 2D poses per tag ID. */
    private final Map<Integer, TrigPoseRecord> trigPoses = new HashMap<>();

    /**
     * Computes a robot pose from a single AprilTag observation using trigonometric relationships.
     *
     * <p>
     * This method estimates the robot’s position in the field by combining the tag’s known
     * location, the measured distance, yaw, and pitch from the camera, and the robot’s current
     * heading. This approach avoids PnP solving and is useful for close-range or single-tag
     * detections where ambiguity is low.
     *
     * @param camera The camera model containing intrinsic and extrinsic parameters.
     * @param observation The detected AprilTag observation.
     * @param heading The robot’s current field-relative heading.
     * @return An {@link Optional} containing the estimated robot pose, or empty if invalid.
     */
    private Optional<Pose2d> solveTrigPosition(Camera camera, TagObservation observation,
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
     * Triangulates and records a single-tag 2D pose observation.
     *
     * <p>
     * Used internally to compute and store a {@link TrigPoseRecord} for each tag observed during a
     * vision update. The results are cached for later retrieval.
     *
     * @param camera The camera that observed the tag.
     * @param timestamp The timestamp of the vision frame.
     * @param observation The AprilTag observation to process.
     * @param heading The robot’s current heading at the observation time.
     */
    private void add2DVisionObservation(Camera camera, Time timestamp,
        TagObservation observation, Rotation2d heading) {
        Optional<Pose2d> pose = solveTrigPosition(camera, observation, heading);
        pose.ifPresent(p -> trigPoses.put(observation.id(),
            new TrigPoseRecord(p, observation.distance(), timestamp)));
    }

    /**
     * Generates an initial seed pose estimate for the constrained PnP solver using the available
     * vision data.
     *
     * <p>
     * This method attempts to derive a reasonable starting pose for PnP optimization based on the
     * detected tags and their known field locations. If a multi-tag estimate is available from the
     * vision system, it is used directly as the seed. Otherwise, a single-tag-based geometric
     * estimate is computed using the tag’s known field pose, the camera-to-tag transform, and the
     * camera’s extrinsics.
     *
     * <p>
     * Providing a good seed improves the convergence and accuracy of the constrained PnP solver,
     * especially in multi-tag scenarios or when tags are at oblique viewing angles.
     *
     * @param observation The {@link VisionObservation} containing detected tags and camera data.
     * @return An {@link Optional} containing the estimated robot pose seed in field coordinates, or
     *         empty if insufficient data is available or the tag pose is unknown.
     */
    private Optional<Pose3d> getConstrainedSolvePnPSeedFromVisionObservation(
        VisionObservation observation) {
        Transform3d cameraToRobot = observation.camera().robotToCamera().inverse();

        if (observation.multiTagCameraPose().isPresent()) {
            Pose3d robotPose = observation.multiTagCameraPose().get().plus(cameraToRobot);
            return Optional.of(robotPose);
        }

        if (observation.tagObservations().isEmpty()) {
            return Optional.empty();
        }
        TagObservation bestTagObservation = observation.tagObservations().get(0);

        if (bestTagObservation.ambiguity() > ambiguityThreshold) {
            return Optional.empty();
        }

        int tagID = bestTagObservation.id();
        var optionalTagPose = fieldLayout.getTagPose(tagID);
        if (optionalTagPose.isEmpty()) {
            return Optional.empty();
        }
        Pose3d tagPose = optionalTagPose.get();

        Transform3d targetToCamera = bestTagObservation.cameraToTarget().inverse();
        Pose3d robotPose = tagPose.plus(targetToCamera).plus(cameraToRobot);

        return Optional.of(robotPose);
    }

    /**
     * Processes a complete set of vision tag detections and computes a 3D pose estimate if valid.
     *
     * <p>
     * This method performs several key steps:
     * <ol>
     * <li>Generates single-tag triangulated poses for all detections.</li>
     * <li>Rejects observations with invalid data.</li>
     * <li>Uses PhotonVision’s constrained PnP solver to estimate the robot’s full 3D pose.</li>
     * <li>Applies filtering and field boundary checks to reject impossible results.</li>
     * <li>Computes uncertainty values based on distance and tag count.</li>
     * </ol>
     *
     * @param observation The {@link VisionObservation} containing all AprilTag detections.
     * @param heading The robot’s field-relative heading at the observation time.
     * @return An {@link Optional} containing a {@link PNPPoseRecord} with the estimated pose and
     *         associated uncertainty, or empty if the observation is invalid or unreliable.
     */
    public Optional<PNPPoseRecord> addVisionObservation(VisionObservation observation,
        Rotation2d heading) {
        var tags = observation.tagObservations();
        Camera camera = observation.camera();
        int tagCount = tags.size();

        // Compute and store triangulated 2D poses for each tag
        for (TagObservation tagObservation : tags) {
            add2DVisionObservation(camera, observation.timestamp(), tagObservation, heading);
        }

        // Ignore invalid observations
        if (tagCount == 0) {
            return Optional.empty();
        }

        // Solve for robot pose using constrained PnP
        var photonTargets = tags.stream().map(TagObservation::toPhotonTarget).toList();
        var robotToCamera = camera.robotToCamera();

        // Attempt to extract seed pose from the observation
        var optionalSeed = getConstrainedSolvePnPSeedFromVisionObservation(observation);
        if (optionalSeed.isEmpty()) {
            return Optional.empty();
        }
        Pose3d seed = optionalSeed.get();

        Optional<Pose3d> optionalEstimate =
            VisionEstimation.estimateRobotPoseConstrainedSolvepnp(
                camera.cameraMatrix(),
                camera.distCoeffs(),
                photonTargets,
                robotToCamera,
                seed,
                fieldLayout,
                TargetModel.kAprilTag36h11,
                false,
                heading,
                gyroHeadingScaleFactor)
                .map(estimate -> GeomUtil.toPose3d(estimate.best));

        if (optionalEstimate.isEmpty()) {
            return Optional.empty();
        }

        Pose3d estimate = optionalEstimate.get();

        // Reject poses that exceed field boundaries or height constraints
        double x = estimate.getX();
        double y = estimate.getY();
        double z = Math.abs(estimate.getZ());
        double fieldLength = fieldLayout.getFieldLength();
        double fieldWidth = fieldLayout.getFieldWidth();

        if (z > maxZMeters || x < 0.0 || x > fieldLength || y < 0.0 || y > fieldWidth) {
            return Optional.empty();
        }

        // Compute distance-based uncertainty scaling
        double avgDistance = tags.stream()
            .mapToDouble(tag -> tag.distance().in(Meters))
            .average().orElse(0.0);

        double stdDevFactor = Math.pow(avgDistance, 2.0) / tagCount;
        double linearStdDev = linearStdDevFactor * stdDevFactor;
        double angularStdDev = angularStdDevFactor * stdDevFactor;

        return Optional.of(new PNPPoseRecord(estimate, linearStdDev, angularStdDev));
    }

    /**
     * Retrieves the most recent triangulation-based pose estimate for a specific AprilTag.
     *
     * <p>
     * This uses trigonometry from a single-tag observation and the robot’s historical odometry to
     * compute the estimated robot pose at the time of observation. This is often more accurate than
     * the global pose in close proximity high-ambiguity situations.
     *
     * @param tagId the AprilTag ID
     * @return an {@link Optional} {@link TrigPoseRecord} containing the pose if valid; otherwise
     *         empty
     */
    public Optional<TrigPoseRecord> getTrigPose(int tagId) {
        return Optional.ofNullable(trigPoses.get(tagId));
    }
}
