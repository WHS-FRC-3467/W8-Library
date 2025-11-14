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

package frc.lib.devices;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.lib.io.vision.photonvision.VisionIOPhotonVision;
import frc.lib.io.vision.photonvision.VisionIOPhotonVision.VisionIOPhotonVisionInputs;
import frc.lib.util.GeomUtil;

public class AprilTagCamera {
    public static record CameraProperties(
        String name,
        Transform3d robotToCamera,
        Matrix<N3, N3> cameraMatrix,
        Matrix<N8, N1> distCoeffs,
        int resolutionWidth,
        int resolutionHeight,
        double stdDevFactor) {
    }

    public static record TagObservation(
        int id,
        double area,
        Angle pitch,
        Angle yaw,
        List<TargetCorner> targetCorners,
        Transform3d cameraToTarget,
        double ambiguity,
        Distance distance) {
        public TagObservation(
            int id,
            double area,
            Angle pitch,
            Angle yaw,
            List<TargetCorner> targetCorners,
            Transform3d cameraToTarget,
            double ambiguity)
        {
            this(
                id,
                area,
                pitch,
                yaw,
                targetCorners,
                cameraToTarget,
                ambiguity,
                Meters.of(cameraToTarget.getTranslation().getNorm()));
        }

        public static TagObservation fromPhotonTarget(PhotonTrackedTarget target)
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

        public PhotonTrackedTarget toPhotonTarget()
        {
            return new PhotonTrackedTarget(
                yaw.in(Degrees),
                pitch.in(Degrees),
                area,
                0,
                id,
                0,
                0,
                cameraToTarget,
                null,
                0,
                null,
                targetCorners);
        }
    }

    public static record VisionObservation(
        Time timestamp,
        CameraProperties camera,
        Optional<Pose3d> multiTagPose,
        List<TagObservation> tagObservations) {
        public static VisionObservation fromPhotonPipelineResult(PhotonPipelineResult result,
            CameraProperties camera)
        {
            List<TagObservation> tagObservations =
                result.getTargets().stream().map(TagObservation::fromPhotonTarget).toList();

            Optional<Pose3d> multiTagPose =
                result.getMultiTagResult()
                    .map(multiTagResult -> GeomUtil.toPose3d(
                        multiTagResult.estimatedPose.best.plus(camera.robotToCamera().inverse())));

            return new VisionObservation(
                Seconds.of(result.getTimestampSeconds()),
                camera,
                multiTagPose,
                tagObservations);
        }
    }

    private final CameraProperties properties;
    private final VisionIOPhotonVision io;
    private final VisionIOPhotonVisionInputs inputs;

    private final Consumer<VisionObservation> visionConsumer;

    public AprilTagCamera(
        CameraProperties properties,
        VisionIOPhotonVision io,
        Consumer<VisionObservation> visionConsumer)
    {
        this.properties = properties;
        this.io = io;
        this.visionConsumer = visionConsumer;
        inputs = new VisionIOPhotonVisionInputs(properties.cameraMatrix(), properties.distCoeffs());
    }

    public void periodic()
    {
        io.updateInputs(inputs);
        Logger.processInputs(properties.name(), inputs);

        if (!inputs.connected)
            return;

        Stream.of(inputs.results)
            .map(result -> VisionObservation.fromPhotonPipelineResult(result, properties))
            .forEach(visionConsumer);
    }
}
