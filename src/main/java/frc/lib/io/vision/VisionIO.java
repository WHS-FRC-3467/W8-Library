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
 * Hardware interface for one vision <em>source</em>: a group of cameras whose AprilTag results
 * arrive through one interface.
 *
 * <ul>
 *   <li>A C2 coprocessor is one source serving all of its cameras, with its own health signal.
 *   <li>PhotonVision exposes only cameras on NetworkTables (which coprocessor runs which camera is
 *       hidden), so a PhotonVision source is simply whichever cameras are grouped together.
 * </ul>
 *
 * <p>The IO only moves raw bytes, so AdvantageKit can log and replay them. Turning bytes into
 * {@link CameraResult}s is done by {@link C2Decoder} or {@link PhotonDecoder}, chosen by the logged
 * {@link NativePacketType}. Everything that differs between kinds of source (packet type, whether
 * it reports its own health) is in the logged inputs, so a log replays correctly whichever kind of
 * source recorded it.
 */
public interface VisionIO {
    /** The format of the raw results in {@link VisionIOInputs#rawResults}. */
    public static enum NativePacketType {
        UNKNOWN,
        C2,
        PHOTON
    }

    /** One AprilTag seen alone: two candidate camera poses (single-tag solves are ambiguous). */
    public static record TagObservation(
            int fiducialId,
            Pose3d fieldToCameraPose, // Field to camera
            Optional<Pose3d> altPose,
            double area,
            double ambiguity) {}

    /** Several AprilTags solved together: one unambiguous camera pose. */
    public static record MultiTagObservation(
            int[] fiducialIds,
            Pose3d fieldToCameraPose, // Field to camera
            double error) {}

    /** Everything one camera reported for one frame. */
    public static record CameraResult(
            TagObservation[] tagObservations,
            Optional<MultiTagObservation> multiTagObservation,
            double captureTimestampUs,
            double publishTimestampUs) {}

    /** Raw readings from one source, logged by AdvantageKit for replay. */
    @AutoLog
    public static class VisionIOInputs {
        /** The format of {@link #rawResults}, which decides how they are decoded. */
        public NativePacketType rawPacketType = NativePacketType.UNKNOWN;

        /**
         * Whether the source has a health signal of its own (C2 does; PhotonVision doesn't), so a
         * disconnect can be blamed on the source rather than on each camera.
         */
        public boolean reportsSourceHealth = false;

        /**
         * Whether the source itself is reachable. For sources without a health signal of their own
         * (PhotonVision), this is always true and only per-camera status is meaningful.
         */
        public boolean connected = false;

        /** Whether each camera of the source is delivering images, indexed by camera. */
        public boolean[] camerasConnected = new boolean[0];

        /** Which camera (index into the source's camera list) each raw result came from. */
        public int[] resultCameras = new int[0];

        // Raw bytes are logged (rather than decoded records) because it is faster and many
        // structured classes cannot be @AutoLog-ed, such as records containing arrays.
        /** Raw unread results since the last update, oldest first per camera. */
        public byte[][] rawResults = new byte[0][];

        /** Capture timestamp of each result, on the robot's clock (FPGA time), microseconds. */
        public long[] captureTimestampsUs = new long[0];

        /** When each result was published or received, robot clock, microseconds. */
        public long[] publishTimestampsUs = new long[0];
    }

    /**
     * Reads everything the source has produced since the last call.
     *
     * @param inputs the inputs to fill
     */
    public default void updateInputs(VisionIOInputs inputs) {}
}
