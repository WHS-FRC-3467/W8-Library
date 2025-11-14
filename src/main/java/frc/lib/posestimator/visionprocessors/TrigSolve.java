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
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.devices.apriltagcameras.AprilTagCamera.CameraProperties;
import frc.lib.devices.apriltagcameras.AprilTagCamera.TagObservation;
import frc.lib.devices.apriltagcameras.AprilTagCamera.TagObservation.VisionObservation;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import lombok.experimental.Accessors;

@RequiredArgsConstructor
@Accessors(fluent = true)
public class TrigSolve implements VisionProcessor {

    private static final double DEFAULT_LINEAR_STDDEV_FACTOR = 0.4;

    private final AprilTagFieldLayout fieldLayout;

    @Getter
    @Setter
    private double linearStdDevFactor = DEFAULT_LINEAR_STDDEV_FACTOR;

    @Getter
    @Setter
    private int followedAprilTag = 0;

    private Optional<Pose2d> solveTrigPosition(
        CameraProperties camera,
        TagObservation target,
        Rotation2d heading)
    {
        Translation2d camToTagTranslation =
            new Translation3d(
                target.cameraToTarget().getTranslation().getNorm(),
                new Rotation3d(
                    0,
                    -target.pitch().in(Radians),
                    -target.yaw().in(Radians)))
                        .rotateBy(camera.robotToCamera().getRotation())
                        .toTranslation2d()
                        .rotateBy(heading);

        var tagPoseOpt = fieldLayout.getTagPose(target.id());
        if (tagPoseOpt.isEmpty()) {
            return Optional.empty();
        }
        var tagPose2d = tagPoseOpt.get().toPose2d();

        Translation2d fieldToCameraTranslation =
            tagPose2d.getTranslation().plus(camToTagTranslation.unaryMinus());

        Translation2d camToRobotTranslation =
            camera.robotToCamera().getTranslation().toTranslation2d().unaryMinus()
                .rotateBy(heading);

        Pose2d robotPose =
            new Pose2d(fieldToCameraTranslation.plus(camToRobotTranslation), heading);

        return Optional.of(robotPose);
    }

    @Override
    public Optional<PoseRecord> processVisionObservation(
        VisionObservation observation,
        Rotation2d heading)
    {
        var camera = observation.camera();
        var tagObservations = observation.tagObservations();

        // Nothing to go off of
        if (tagObservations.isEmpty()) {
            return Optional.empty();
        }

        // The observation that matches the tag ID we're looking for
        var optionalWantedObservation = tagObservations.stream()
            .filter(target -> target.id() == followedAprilTag)
            .findFirst();

        // It wasn't found
        if (optionalWantedObservation.isEmpty()) {
            return Optional.empty();
        }

        TagObservation wantedObservation = optionalWantedObservation.get();

        double stdDevFactor =
            Math.pow(wantedObservation.distance().in(Meters), 2)
                * camera.stdDevFactor();
        double linearStdDev = linearStdDevFactor * stdDevFactor;

        // This processor assumes supplied heading is perfect
        double angularStdDev = Double.POSITIVE_INFINITY;

        return solveTrigPosition(camera, wantedObservation, heading)
            .map(p -> new PoseRecord(new Pose3d(p), linearStdDev, angularStdDev));
    }
}
