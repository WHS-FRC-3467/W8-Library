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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import frc.lib.io.vision.VisionIO.CameraResult;
import frc.lib.io.vision.VisionIO.MultiTagObservation;
import frc.lib.io.vision.VisionIO.TagObservation;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Optional;

/**
 * Decodes C2's AprilTag pipeline output: one {@code dsv0.Frame} per camera per frame (schema
 * {@code C2/schema/apriltag.fbs}), already solved to a field-to-camera pose on the coprocessor.
 */
public class C2Decoder {

    private final AprilTagFieldLayout tagLayout;

    /**
     * @param tagLayout field layout, to check observed tag IDs
     */
    public C2Decoder(AprilTagFieldLayout tagLayout) {
        this.tagLayout = tagLayout;
    }

    /**
     * Decodes one result.
     *
     * @param raw the raw bytes
     * @param captureTimestampUs capture time, robot clock (FPGA), microseconds
     * @param publishTimestampUs publish/receive time, robot clock, microseconds
     * @return the result, or {@code null} if {@code raw} isn't valid
     */
    public CameraResult decode(byte[] raw, long captureTimestampUs, long publishTimestampUs) {
        if (raw == null || raw.length == 0) {
            return null;
        }
        dsv0.Frame frame;
        try {
            frame = dsv0.Frame.getRootAsFrame(ByteBuffer.wrap(raw));
        } catch (RuntimeException e) {
            return null;
        }

        // Each message carries exactly one camera's output (one NT topic per camera).
        dsv0.CameraOutput output = frame.camerasLength() > 0 ? frame.cameras(0) : null;
        dsv0.CameraObservation observation = output == null ? null : output.cameraObservation();
        if (observation == null || observation.solution0() == null) {
            return empty(captureTimestampUs, publishTimestampUs);
        }

        dsv0.PoseSolution best = observation.solution0();
        Pose3d fieldToCamera = toWpilibPose(best);

        // Single tag: two candidate poses (IPPE); report both with an ambiguity ratio.
        ArrayList<TagObservation> tagObservations = new ArrayList<>(1);
        if (observation.tagIdsLength() == 1) {
            int tagId = observation.tagIds(0);
            if (tagLayout.getTagPose(tagId).isPresent()) {
                Optional<dsv0.PoseSolution> alternate =
                        Optional.ofNullable(observation.solution1());
                tagObservations.add(
                        new TagObservation(
                                tagId,
                                fieldToCamera,
                                alternate.map(C2Decoder::toWpilibPose),
                                0.0,
                                ambiguity(
                                        best.error(),
                                        alternate.map(dsv0.PoseSolution::error).orElse(-1.0))));
            }
        }

        // Two or more tags: one unambiguous pose solved from every corner.
        Optional<MultiTagObservation> multiTag = Optional.empty();
        if (observation.tagIdsLength() >= 2) {
            int[] ids = new int[observation.tagIdsLength()];
            for (int i = 0; i < ids.length; i++) {
                ids[i] = observation.tagIds(i);
            }
            multiTag = Optional.of(new MultiTagObservation(ids, fieldToCamera, best.error()));
        }

        return new CameraResult(
                tagObservations.toArray(TagObservation[]::new),
                multiTag,
                (double) captureTimestampUs,
                (double) publishTimestampUs);
    }

    private static CameraResult empty(long captureTimestampUs, long publishTimestampUs) {
        return new CameraResult(
                new TagObservation[0],
                Optional.empty(),
                (double) captureTimestampUs,
                (double) publishTimestampUs);
    }

    private static Pose3d toWpilibPose(dsv0.PoseSolution solution) {
        dsv0.Pose3d pose = solution.pose();
        dsv0.Vec3 t = pose.translation();
        dsv0.Quaternion q = pose.rotation();
        return new Pose3d(
                new Translation3d(t.x(), t.y(), t.z()),
                new Rotation3d(
                        new edu.wpi.first.math.geometry.Quaternion(q.w(), q.x(), q.y(), q.z())));
    }

    private static double ambiguity(double bestError, double alternateError) {
        if (alternateError <= 0.0) return 0.0;
        return Math.max(0.0, Math.min(1.0, bestError / alternateError));
    }
}
