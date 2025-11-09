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

import edu.wpi.first.math.Matrix;
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
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean connected = false;
        public VisionObservation[] poseObservations = null;
    }

    /**
     * Represents a single vision camera and its calibration data.
     *
     * <p>
     * This record contains information describing the physical and optical configuration of a
     * camera used for vision processing:
     * <ul>
     * <li>{@code name} — the identifier or nickname of the camera</li>
     * <li>{@code robotToCamera} — the transform from the robot coordinate frame to the camera's
     * coordinate frame</li>
     * <li>{@code cameraMatrix} — the intrinsic camera calibration matrix (3×3)</li>
     * <li>{@code distCoeffs} — the distortion coefficients (8×1) used to correct lens
     * distortion</li>
     * <li>{@code resolutionWidth} — the horizontal resolution of the camera image in pixels</li>
     * <li>{@code resultionHeight} — the vertical resolution of the camera image in pixels</li>
     * </ul>
     *
     * <p>
     * This record is typically used to describe the configuration of cameras for pose estimation or
     * AprilTag detection.
     *
     * @param name the name or identifier of the camera
     * @param robotToCamera the transform from robot frame to camera frame
     * @param cameraMatrix the intrinsic camera matrix (3×3)
     * @param distCoeffs the distortion coefficients (8×1)
     * @param resolutionWidth the image width in pixels
     * @param resultionHeight the image height in pixels
     */
    public record Camera(
        String name,
        Transform3d robotToCamera,
        Matrix<N3, N3> cameraMatrix,
        Matrix<N8, N1> distCoeffs,
        int resolutionWidth,
        int resultionHeight) {
    }

    /**
     * Represents a single detected AprilTag or fiducial target in an image.
     *
     * <p>
     * This record encapsulates all relevant information returned by vision processing for a single
     * tag, including its geometry, estimated pose, and detection quality:
     * <ul>
     * <li>{@code id} — the numeric ID of the detected tag</li>
     * <li>{@code area} — the percentage of the image occupied by the tag</li>
     * <li>{@code pitch} — the vertical angle to the tag (positive up)</li>
     * <li>{@code yaw} — the horizontal angle to the tag (positive left)</li>
     * <li>{@code targetCorners} — the 2D image-space coordinates of the tag corners</li>
     * <li>{@code cameraToTarget} — the transform from camera to the detected tag in 3D space</li>
     * <li>{@code ambiguity} — the ambiguity factor of the pose solution (lower is better)</li>
     * <li>{@code distance} — the estimated straight-line distance from the camera to the tag</li>
     * </ul>
     *
     * <p>
     * The alternate constructor automatically computes the distance using the translation magnitude
     * of the {@code cameraToTarget} transform.
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
                null,
                null,
                0,
                null,
                targetCorners);
        }
    }

    /**
     * Represents a single vision-based pose observation for the robot.
     *
     * <p>
     * This record encapsulates all relevant data from a camera frame used in pose estimation:
     * <ul>
     * <li>{@code timestamp} — the time at which the observation was captured</li>
     * <li>{@code camera} — the camera that produced this observation</li>
     * <li>{@code multiTagCameraPose} — an optional pose estimate of the camera in the field
     * coordinate frame, computed from multiple tags</li>
     * <li>{@code tagObservations} — the list of all individual tag detections in this frame</li>
     * </ul>
     *
     * <p>
     * Instances of this record are typically passed to pose estimators or sensor fusion algorithms
     * to incorporate vision data into the robot’s field-relative pose estimate.
     *
     * @param timestamp the capture time of the observation
     * @param camera the camera that generated this observation
     * @param multiTagCameraPose optional estimated pose of the camera using multiple tags
     * @param tagObservations the list of detected tags in this frame
     */
    public static record VisionObservation(
        Time timestamp,
        Camera camera,
        Optional<Pose3d> multiTagCameraPose,
        List<TagObservation> tagObservations) {
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}
