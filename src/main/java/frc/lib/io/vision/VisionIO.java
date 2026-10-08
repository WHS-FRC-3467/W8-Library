/*
 * Copyright (C) 2026 Windham Windup
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

import edu.wpi.first.math.geometry.Pose3d;

import org.littletonrobotics.junction.AutoLog;

import java.util.Optional;

/**
 * Hardware interface for vision cameras that detect AprilTags for robot localization.
 *
 * <p>This interface defines the contract for vision camera hardware, allowing the robot to read
 * camera results for pose estimation. Implementations handle vendor-specific camera APIs
 * (PhotonVision, Limelight, etc.) while the rest of the robot code remains hardware-agnostic.
 */
public interface VisionIO {
    /** Identifier for the type of raw packet we are recieving in {@link VisionIOInputs#} */
    public static enum NativePacketType {
        UNKNOWN,
        C2,
        PHOTON
    }

    /**
     * Container for vision camera sensor readings. Logged automatically by AdvantageKit for replay
     * and analysis.
     */
    public static record TagObservation(
            int fiducialId,
            Pose3d fieldToCameraPose, // Field to camera
            Optional<Pose3d> altPose,
            double area,
            double ambiguity) {}

    public static record MultiTagObservation(
            int[] fiducialIds,
            Pose3d fieldToCameraPose, // Field to camera
            double error) {}

    /** A pose already solved for the robot frame by a multi-camera vision source. */
    public static record RobotPoseObservation(
            int[] fiducialIds, Pose3d fieldToRobotPose, double reprojectionError) {}

    public static record CameraResult(
            TagObservation[] tagObservations,
            Optional<MultiTagObservation> multiTagObservation,
            double captureTimestampUs,
            double publishTimestampUs,
            int cameraIndex,
            Optional<RobotPoseObservation> robotPoseObservation) {
        public CameraResult(
                TagObservation[] tagObservations,
                Optional<MultiTagObservation> multiTagObservation,
                double captureTimestampUs,
                double publishTimestampUs) {
            this(
                    tagObservations,
                    multiTagObservation,
                    captureTimestampUs,
                    publishTimestampUs,
                    -1,
                    Optional.empty());
        }

        public CameraResult(
                TagObservation[] tagObservations,
                Optional<MultiTagObservation> multiTagObservation,
                double captureTimestampUs,
                double publishTimestampUs,
                Optional<RobotPoseObservation> robotPoseObservation) {
            this(
                    tagObservations,
                    multiTagObservation,
                    captureTimestampUs,
                    publishTimestampUs,
                    -1,
                    robotPoseObservation);
        }

        public CameraResult(
                TagObservation[] tagObservations,
                Optional<MultiTagObservation> multiTagObservation,
                double captureTimestampUs,
                double publishTimestampUs,
                int cameraIndex) {
            this(
                    tagObservations,
                    multiTagObservation,
                    captureTimestampUs,
                    publishTimestampUs,
                    cameraIndex,
                    Optional.empty());
        }
    }

    @AutoLog
    public static class VisionIOInputs {
        /** Whether the camera is connected and responding */
        public boolean connected = false;

        /** The type of packet in {@link VisionIOInputs#rawResults} */
        public NativePacketType rawPacketType = NativePacketType.UNKNOWN;

        /** Raw unread frame payloads from the camera since last update. */
        // We log raw bytes both because it is faster and many structured
        // classes cannot be @AutoLog-ed, such as records containing
        // arrays
        public byte[][] rawResults = new byte[0][];

        /** NT-synced capture timestamps for each unread result, in microseconds. */
        public long[] captureTimestampsUs = new long[0];

        /** NT-synced publish timestamps for each unread result, in microseconds. */
        public long[] publishTimestampsUs = new long[0];
    }

    /**
     * Updates the vision inputs with the latest readings from the camera. Called periodically by
     * the vision device layer.
     *
     * @param inputs The input object to populate with sensor data
     */
    public default void updateInputs(VisionIOInputs inputs) {}
}
