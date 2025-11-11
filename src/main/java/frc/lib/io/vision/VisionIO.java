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

package frc.lib.io.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public interface VisionIO {
    public class VisionIOInputs implements LoggableInputs {
        public boolean connected = false;
        public VisionIO.VisionObservation[] poseObservations = new VisionIO.VisionObservation[0];

        @Override
        public void toLog(LogTable table) {
            table.put("Connected", connected);
            table.put("ObservationCount", poseObservations.length);

            for (int i = 0; i < poseObservations.length; i++) {
                var obs = poseObservations[i];
                var prefix = "Observations/" + i + "/";

                table.put(prefix + "Timestamp", obs.timestamp.in(Seconds));

                // Camera info
                var cam = obs.camera();
                var camPrefix = prefix + "Camera/";
                table.put(camPrefix + "Name", cam.name());
                table.put(camPrefix + "ResWidth", cam.resolutionWidth());
                table.put(camPrefix + "ResHeight", cam.resolutionHeight());
                table.put(camPrefix + "RobotToCamera", cam.robotToCamera());

                // Intrinsics
                table.put(camPrefix + "CameraMatrix", cam.cameraMatrix().getData());
                table.put(camPrefix + "DistCoeffs", cam.distCoeffs().getData());

                // Optional multi-tag pose
                if (obs.multiTagCameraPose().isPresent()) {
                    table.put(prefix + "HasMultiPose", true);
                    table.put(prefix + "MultiPose", obs.multiTagCameraPose().get());
                } else {
                    table.put(prefix + "HasMultiPose", false);
                }

                // Tag observations
                var tags = obs.tagObservations();
                table.put(prefix + "TagCount", tags.size());
                for (int j = 0; j < tags.size(); j++) {
                    var tag = tags.get(j);
                    var tagPrefix = prefix + "Tags/" + j + "/";

                    table.put(tagPrefix + "ID", tag.id());
                    table.put(tagPrefix + "Area", tag.area());
                    table.put(tagPrefix + "PitchDeg", tag.pitch().in(Degrees));
                    table.put(tagPrefix + "YawDeg", tag.yaw().in(Degrees));
                    table.put(tagPrefix + "Ambiguity", tag.ambiguity());
                    table.put(tagPrefix + "Distance", tag.distance().in(Meters));
                    table.put(tagPrefix + "CameraToTarget", tag.cameraToTarget());

                    var corners = tag.targetCorners();
                    table.put(tagPrefix + "CornerCount", corners.size());
                    for (int k = 0; k < corners.size(); k++) {
                        var corner = corners.get(k);
                        var cornerPrefix = tagPrefix + "Corners/" + k + "/";

                        table.put(cornerPrefix + "x", corner.x);
                        table.put(cornerPrefix + "y", corner.y);
                    }
                }
            }
        }

        @Override
        public void fromLog(LogTable table) {
            connected = table.get("Connected", false);
            int obsCount = table.get("ObservationCount", 0);
            poseObservations = new VisionIO.VisionObservation[obsCount];

            for (int i = 0; i < obsCount; i++) {
                var prefix = "Observations/" + i + "/";

                Time timestamp = Seconds.of(table.get(prefix + "Timestamp", 0.0));

                // Camera reconstruction
                var camPrefix = prefix + "Camera/";
                String name = table.get(camPrefix + "Name", "");
                int width = table.get(camPrefix + "ResWidth", 0);
                int height = table.get(camPrefix + "ResHeight", 0);
                Transform3d robotToCamera =
                    table.get(camPrefix + "RobotToCamera", new Transform3d());

                double[][] camMatrixData = table.get(camPrefix + "CameraMatrix", new double[3][3]);
                double[][] distCoeffsData = table.get(camPrefix + "DistCoeffs", new double[8][1]);

                Matrix<N3, N3> camMatrix = MatBuilder.fill(Nat.N3(), Nat.N3(),
                    Arrays.stream(camMatrixData).flatMapToDouble(Arrays::stream).toArray());
                Matrix<N8, N1> distCoeffs = MatBuilder.fill(Nat.N8(), Nat.N1(),
                    Arrays.stream(distCoeffsData).flatMapToDouble(Arrays::stream).toArray());

                VisionIO.CameraProperties camera =
                    new VisionIO.CameraProperties(name, robotToCamera, camMatrix, distCoeffs, width,
                        height);

                // Optional multi-tag pose
                boolean hasMultiPose = table.get(prefix + "HasMultiPose", false);
                Optional<Pose3d> multiPose = Optional.ofNullable(
                    hasMultiPose ? table.get(prefix + "MultiPose", new Pose3d()) : null);

                // Tag observations
                int tagCount = table.get(prefix + "TagCount", 0);
                List<VisionIO.TagObservation> tags = new ArrayList<>();
                for (int j = 0; j < tagCount; j++) {
                    var tagPrefix = prefix + "Tags/" + j + "/";
                    int id = table.get(tagPrefix + "ID", -1);
                    double area = table.get(tagPrefix + "Area", 0.0);
                    Angle pitch = Degrees.of(table.get(tagPrefix + "PitchDeg", 0.0));
                    Angle yaw = Degrees.of(table.get(tagPrefix + "YawDeg", 0.0));
                    double ambiguity = table.get(tagPrefix + "Ambiguity", 0.0);
                    Distance distance = Meters.of(table.get(tagPrefix + "Distance", 0.0));
                    Transform3d camToTarget =
                        table.get(tagPrefix + "CameraToTarget", new Transform3d());

                    int cornerCount = table.get(tagPrefix + "CornerCount", 0);
                    List<TargetCorner> corners = new ArrayList<>();
                    for (int k = 0; k < cornerCount; k++) {
                        var cornerPrefix = tagPrefix + "Corners/" + k + "/";

                        double x = table.get(cornerPrefix + "x", 0.0);
                        double y = table.get(cornerPrefix + "y", 0.0);

                        corners.add(new TargetCorner(x, y));
                    }

                    tags.add(new VisionIO.TagObservation(
                        id, area, pitch, yaw, corners, camToTarget, ambiguity, distance));
                }

                poseObservations[i] =
                    new VisionIO.VisionObservation(timestamp, camera, multiPose, tags);
            }
        }
    }


    /**
     * Represents a single vision camera and its calibration data.
     *
     * @param name The name or identifier of the camera
     * @param robotToCamera The transform from robot frame to camera frame
     * @param cameraMatrix The intrinsic camera matrix (3×3)
     * @param distCoeffs The distortion coefficients (8×1)
     * @param resolutionWidth The image width in pixels
     * @param resolutionHeight The image height in pixels
     */
    public record CameraProperties(
        String name,
        Transform3d robotToCamera,
        Matrix<N3, N3> cameraMatrix,
        Matrix<N8, N1> distCoeffs,
        int resolutionWidth,
        int resolutionHeight) {
    }

    /**
     * Represents a single detected AprilTag or fiducial target in an image.
     *
     * <p>
     * This record encapsulates all relevant information returned by vision processing for a single
     * tag, including its geometry, estimated pose, and detection quality:
     *
     * @param id the tag ID
     * @param area the fractional image area occupied by the tag
     * @param pitch the vertical viewing angle to the tag
     * @param yaw the horizontal viewing angle to the tag
     * @param targetCorners the detected 2D tag corners in image coordinates
     * @param cameraToTarget the transform from the camera to the tag
     * @param ambiguity the ambiguity score for the pose estimate
     * @param distance the estimated distance to the tag
     */
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
            double ambiguity) {
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

        /**
         * Converts this observation into a {@link PhotonTrackedTarget} for compatibility with
         * PhotonVision APIs.
         *
         * @return a new {@code PhotonTrackedTarget} representing this tag observation
         */
        public PhotonTrackedTarget toPhotonTarget() {
            return new PhotonTrackedTarget(
                pitch.in(Degrees),
                yaw.in(Degrees),
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

    /**
     * Represents a single vision-based pose observation for the robot.
     *
     * @param timestamp the capture time of the observation
     * @param camera the camera that generated this observation
     * @param multiTagCameraPose optional estimated pose of the camera using multiple tags
     * @param tagObservations the list of detected tags in this frame
     */
    public static record VisionObservation(
        Time timestamp,
        CameraProperties camera,
        Optional<Pose3d> multiTagCameraPose,
        List<TagObservation> tagObservations) {
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}
