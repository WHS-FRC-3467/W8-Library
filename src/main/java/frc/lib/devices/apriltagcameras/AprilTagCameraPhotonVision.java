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

package frc.lib.devices.apriltagcameras;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.devices.apriltagcameras.AprilTagCamera.TagObservation.VisionObservation;
import frc.lib.io.vision.photonvision.VisionIOPhotonVision;
import frc.lib.io.vision.photonvision.VisionIOPhotonVision.VisionIOPhotonVisionInputs;
import frc.lib.util.GeomUtil;

public class AprilTagCameraPhotonVision implements AprilTagCamera {
    private final CameraProperties properties;
    private final VisionIOPhotonVision io;
    private final VisionIOPhotonVisionInputs inputs;

    private final Consumer<VisionObservation> visionConsumer;

    public static VisionObservation fromPhotonPipelineResult(PhotonPipelineResult result,
        CameraProperties camera)
    {
        List<TagObservation> tagObservations =
            result.getTargets().stream()
                .map(AprilTagCameraPhotonVision::tagObservationFromPhotonTarget).toList();

        Optional<Pose3d> multiTagPose =
            result.getMultiTagResult()
                .map(multiTagResult -> GeomUtil.toPose3d(
                    multiTagResult.estimatedPose.best
                        .plus(camera.robotToCamera().inverse())));

        return new VisionObservation(
            Seconds.of(result.getTimestampSeconds()),
            camera,
            multiTagPose,
            tagObservations);
    }

    public static PhotonTrackedTarget tagObservationToPhotonTarget(TagObservation observation)
    {
        return new PhotonTrackedTarget(
            observation.yaw().in(Degrees),
            observation.pitch().in(Degrees),
            observation.area(),
            0,
            observation.id(),
            0,
            0,
            observation.cameraToTarget(),
            null,
            0,
            null,
            observation.targetCorners());
    }


    public static TagObservation tagObservationFromPhotonTarget(PhotonTrackedTarget target)
    {
        return new TagObservation(
            target.getFiducialId(),
            target.getArea(),
            Degrees.of(target.getPitch()),
            Degrees.of(target.getYaw()),
            target.getDetectedCorners(),
            target.getBestCameraToTarget(),
            target.getPoseAmbiguity());
    }

    public AprilTagCameraPhotonVision(
        AprilTagCamera.CameraProperties properties,
        VisionIOPhotonVision io,
        Consumer<VisionObservation> visionConsumer)
    {
        this.properties = properties;
        this.io = io;
        this.visionConsumer = visionConsumer;
        inputs = new VisionIOPhotonVisionInputs(properties.cameraMatrix(), properties.distCoeffs());
    }

    @Override
    public void periodic()
    {
        io.updateInputs(inputs);
        Logger.processInputs(properties.name(), inputs);

        if (!inputs.connected)
            return;

        Stream.of(inputs.results)
            .map(result -> fromPhotonPipelineResult(result, properties))
            .forEach(visionConsumer);
    }
}
