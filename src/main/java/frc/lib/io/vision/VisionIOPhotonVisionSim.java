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

import static edu.wpi.first.units.Units.Radians;

import com.google.flatbuffers.FlatBufferBuilder;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.WPIUtilJNI;

import frc.lib.devices.AprilTagCamera.CameraProperties;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Simulates the robot's C2 camera output using PhotonVision's camera simulator.
 *
 * <p>All simulated cameras are owned by one IO instance. Their independent camera-space estimates
 * are serialized into the per-camera FlatBuffers format also supported by C2.
 */
public class VisionIOPhotonVisionSim implements VisionIO {
    private record SimCamera(CameraProperties properties, PhotonCamera camera) {}

    private record SimCameraResult(CameraProperties camera, CameraResult result) {}

    private final Supplier<Pose2d> poseSupplier;
    private final VisionSystemSim system;
    private final AprilTagFieldLayout fieldLayout;
    private final List<SimCamera> cameras = new ArrayList<>();

    public VisionIOPhotonVisionSim(
            CameraProperties[] cameraProperties,
            VisionSystemSim system,
            Supplier<Pose2d> poseSupplier,
            AprilTagFieldLayout fieldLayout) {
        if (cameraProperties == null || cameraProperties.length == 0) {
            throw new IllegalArgumentException("At least one simulated camera is required");
        }
        this.poseSupplier = poseSupplier;
        this.system = system;
        this.fieldLayout = fieldLayout;

        for (CameraProperties properties : cameraProperties) {
            PhotonCamera photonCamera = new PhotonCamera(properties.name());
            var simCameraProperties = new SimCameraProperties();
            if (properties.cameraMatrix() == null || properties.distCoeffs() == null) {
                simCameraProperties.setCalibration(
                        properties.resolutionWidth(),
                        properties.resolutionHeight(),
                        Rotation2d.fromRadians(properties.fov().in(Radians)));
            } else {
                simCameraProperties.setCalibration(
                        properties.resolutionWidth(),
                        properties.resolutionHeight(),
                        properties.cameraMatrix(),
                        properties.distCoeffs());
            }

            simCameraProperties.setFPS(properties.fps());
            simCameraProperties.setAvgLatencyMs(properties.latency().in(Units.Milliseconds));
            simCameraProperties.setLatencyStdDevMs(
                    properties.latencyStdDev().in(Units.Milliseconds));

            PhotonCameraSim cameraSim =
                    new PhotonCameraSim(photonCamera, simCameraProperties, fieldLayout);
            system.addCamera(cameraSim, properties.robotToCamera());
            cameras.add(new SimCamera(properties, photonCamera));
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        system.update(poseSupplier.get());
        inputs.connected = true;
        inputs.rawPacketType = NativePacketType.C2;

        ArrayList<SimCameraResult> results = new ArrayList<>();
        boolean receivedFrame = false;
        for (SimCamera camera : cameras) {
            List<PhotonPipelineResult> unread = camera.camera().getAllUnreadResults();
            if (unread.isEmpty()) continue;
            receivedFrame = true;

            PhotonPipelineResult photon = unread.get(unread.size() - 1);
            long captureTimestampUs = photon.metadata.captureTimestampMicros;
            CameraResult result =
                    VisionIOPhotonVision.toCameraResult(
                            photon,
                            captureTimestampUs,
                            photon.metadata.publishTimestampMicros,
                            fieldLayout,
                            camera.properties().index());
            results.add(new SimCameraResult(camera.properties(), result));
        }

        if (!receivedFrame) {
            inputs.rawResults = new byte[0][];
            inputs.captureTimestampsUs = new long[0];
            inputs.publishTimestampsUs = new long[0];
            return;
        }

        long captureTimestampUs =
                results.stream()
                        .mapToLong(result -> (long) result.result().captureTimestampUs())
                        .max()
                        .orElse(WPIUtilJNI.now());
        long publishTimestampUs = WPIUtilJNI.now();
        inputs.rawResults = new byte[][] {encodePerCameraFrame(results)};
        inputs.captureTimestampsUs = new long[] {captureTimestampUs};
        inputs.publishTimestampsUs = new long[] {publishTimestampUs};
    }

    private static byte[] encodePerCameraFrame(List<SimCameraResult> results) {
        FlatBufferBuilder builder = new FlatBufferBuilder(256);
        int[] cameraOutputs = new int[results.size()];
        for (int i = 0; i < results.size(); i++) {
            SimCameraResult cameraResult = results.get(i);
            int observationOffset = encodeCameraObservation(builder, cameraResult.result());
            cameraOutputs[i] =
                    dsv0.CameraOutput.createCameraOutput(
                            builder,
                            cameraResult.camera().index(),
                            observationOffset,
                            (int) Math.round(cameraResult.camera().fps()));
        }

        int outputsOffset = dsv0.PerCameraResults.createResultsVector(builder, cameraOutputs);
        int resultsOffset = dsv0.PerCameraResults.createPerCameraResults(builder, outputsOffset);
        int frameOffset =
                dsv0.Frame.createFrame(builder, dsv0.Results.PerCameraResults, resultsOffset);
        dsv0.Frame.finishFrameBuffer(builder, frameOffset);
        return builder.sizedByteArray();
    }

    private static int encodeCameraObservation(FlatBufferBuilder builder, CameraResult result) {
        Pose3d pose;
        int[] tagIds;
        double error;
        Pose3d alternatePose = null;
        double alternateError = 0.0;
        if (result.multiTagObservation().isPresent()) {
            MultiTagObservation multiTag = result.multiTagObservation().get();
            pose = multiTag.fieldToCameraPose();
            tagIds = multiTag.fiducialIds();
            error = multiTag.error();
        } else if (result.tagObservations().length > 0) {
            TagObservation best =
                    java.util.Arrays.stream(result.tagObservations())
                            .min(java.util.Comparator.comparingDouble(TagObservation::ambiguity))
                            .orElseThrow();
            pose = best.fieldToCameraPose();
            tagIds = new int[] {best.fiducialId()};
            error = best.ambiguity();
            alternatePose = best.altPose().orElse(null);
            alternateError = 1.0;
        } else {
            return 0;
        }

        int solution0 = encodePoseSolution(builder, pose, error);
        int solution1 =
                alternatePose == null
                        ? 0
                        : encodePoseSolution(builder, alternatePose, alternateError);
        int tagIdsOffset = dsv0.CameraObservation.createTagIdsVector(builder, tagIds);
        return dsv0.CameraObservation.createCameraObservation(
                builder, solution0, solution1, tagIdsOffset);
    }

    private static int encodePoseSolution(
            FlatBufferBuilder builder, Pose3d pose, double reprojectionError) {
        var quaternion = pose.getRotation().getQuaternion();
        int poseOffset =
                dsv0.Pose3d.createPose3d(
                        builder,
                        pose.getX(),
                        pose.getY(),
                        pose.getZ(),
                        quaternion.getW(),
                        quaternion.getX(),
                        quaternion.getY(),
                        quaternion.getZ());
        dsv0.PoseSolution.startPoseSolution(builder);
        dsv0.PoseSolution.addPose(builder, poseOffset);
        dsv0.PoseSolution.addReprojectionError(builder, reprojectionError);
        return dsv0.PoseSolution.endPoseSolution(builder);
    }
}
