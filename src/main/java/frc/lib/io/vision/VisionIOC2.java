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

import dsv0.PoseSolution;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.TimestampedRaw;
import edu.wpi.first.util.WPIUtilJNI;

import frc.lib.devices.AprilTagCamera.CameraProperties;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Supplier;

/**
 * Real hardware implementation of {@link VisionIO} using c2.
 *
 * <p>Publishes shared robot configuration, including all camera extrinsics, and reads the combined
 * robot-pose flatbuffer output once per frame.
 */
public class VisionIOC2 implements VisionIO {
    private static final String FLATBUFFER_TYPE = "dsv0_fb";
    private static final long DISCONNECT_TIMEOUT_US = 500_000L;
    private static final ConcurrentHashMap<String, C2DeviceContext> DEVICE_CONTEXTS =
            new ConcurrentHashMap<>();

    /**
     * Shared capture configuration for c2.
     *
     * @param deviceId NetworkTables device ID root, e.g. {@code dsv0}
     * @param exposure Exposure value sent to c2's config topic
     * @param gain Gain value sent to c2's config topic
     * @param fiducialSizeMeters AprilTag edge length used by c2 solvePnP
     * @param tagLayout Field layout used both for config publishing and result reconstruction
     * @param tagLayoutJson Serialized field layout in c2's expected JSON format
     * @param pollStorageDepth NT queue depth for unread raw frames
     */
    public static record C2Config(
            String deviceId,
            int exposure,
            int gain,
            double fiducialSizeMeters,
            AprilTagFieldLayout tagLayout,
            String tagLayoutJson,
            int pollStorageDepth) {}

    private static record SharedDeviceConfig(
            int exposure,
            int gain,
            List<Pose3d> cameraExtrinsics,
            double fiducialSizeMeters,
            String tagLayoutJson) {}

    /** Per-device shared c2 config publishers. Camera outputs remain per VisionIO instance. */
    private static final class C2DeviceContext {
        private final NetworkTableInstance ntInstance;
        private final String deviceId;
        private final IntegerPublisher exposurePublisher;
        private final IntegerPublisher gainPublisher;
        private final DoublePublisher fiducialSizePublisher;
        private final StringPublisher tagLayoutPublisher;
        private final StructArrayPublisher<Pose3d> cameraExtrinsicsPublisher;
        private final Map<Integer, String> cameraNamesByIndex = new HashMap<>();

        private SharedDeviceConfig sharedConfig = null;
        private boolean hasPublishedConfig = false;
        private boolean lastNtConnected = false;

        private C2DeviceContext(
                NetworkTableInstance ntInstance,
                CameraProperties[] cameraProperties,
                C2Config config) {
            this.ntInstance = ntInstance;
            this.deviceId = config.deviceId();

            NetworkTable configTable = ntInstance.getTable("/" + deviceId + "/config");
            exposurePublisher = configTable.getIntegerTopic("camera_exposure").publish();
            gainPublisher = configTable.getIntegerTopic("camera_gain").publish();
            fiducialSizePublisher = configTable.getDoubleTopic("fiducial_size_m").publish();
            tagLayoutPublisher = configTable.getStringTopic("tag_layout").publish();
            cameraExtrinsicsPublisher =
                    configTable.getStructArrayTopic("camera_extrinsics", Pose3d.struct).publish();

            for (CameraProperties properties : cameraProperties) {
                cameraNamesByIndex.put(properties.index(), properties.name());
            }

            registerCameras(cameraProperties, config);
        }

        public synchronized void registerCameras(
                CameraProperties[] cameraProperties, C2Config config) {
            for (CameraProperties properties : cameraProperties) {
                String existingCameraName =
                        cameraNamesByIndex.putIfAbsent(properties.index(), properties.name());
                if (existingCameraName != null && !existingCameraName.equals(properties.name())) {
                    throw new IllegalArgumentException(
                            "Duplicate c2 cameraIndex "
                                    + properties.index()
                                    + " for deviceId \""
                                    + deviceId
                                    + "\". Cameras \""
                                    + existingCameraName
                                    + "\" and \""
                                    + properties.name()
                                    + "\" cannot share the same output topic.");
                }
            }

            SharedDeviceConfig candidateSharedConfig = sharedConfigFor(cameraProperties, config);
            if (sharedConfig == null) {
                sharedConfig = candidateSharedConfig;
            } else if (!sharedConfig.equals(candidateSharedConfig)) {
                throw conflictingSharedConfig(candidateSharedConfig);
            }

            publishConfigIfNeeded();
        }

        public synchronized void publishConfigIfNeeded() {
            if (sharedConfig == null) {
                return;
            }

            boolean connected = ntInstance.isConnected();
            boolean shouldPublish = !hasPublishedConfig || (connected && !lastNtConnected);
            lastNtConnected = connected;

            if (!shouldPublish) {
                return;
            }

            exposurePublisher.set(sharedConfig.exposure());
            gainPublisher.set(sharedConfig.gain());
            fiducialSizePublisher.set(sharedConfig.fiducialSizeMeters());
            tagLayoutPublisher.set(sharedConfig.tagLayoutJson());
            cameraExtrinsicsPublisher.set(sharedConfig.cameraExtrinsics().toArray(Pose3d[]::new));
            hasPublishedConfig = true;
        }

        private IllegalArgumentException conflictingSharedConfig(
                SharedDeviceConfig candidateSharedConfig) {
            StringBuilder mismatches = new StringBuilder();
            appendMismatch(
                    mismatches,
                    "exposure",
                    sharedConfig.exposure(),
                    candidateSharedConfig.exposure());
            appendMismatch(mismatches, "gain", sharedConfig.gain(), candidateSharedConfig.gain());
            appendMismatch(
                    mismatches,
                    "cameraExtrinsics",
                    sharedConfig.cameraExtrinsics(),
                    candidateSharedConfig.cameraExtrinsics());
            appendMismatch(
                    mismatches,
                    "fiducialSizeMeters",
                    sharedConfig.fiducialSizeMeters(),
                    candidateSharedConfig.fiducialSizeMeters());
            appendMismatch(
                    mismatches,
                    "tagLayoutJson",
                    sharedConfig.tagLayoutJson(),
                    candidateSharedConfig.tagLayoutJson());

            return new IllegalArgumentException(
                    "Conflicting shared c2 config for deviceId \""
                            + deviceId
                            + "\". All cameras publishing to /"
                            + deviceId
                            + "/config must agree on: "
                            + mismatches
                            + ".");
        }

        private static void appendMismatch(
                StringBuilder mismatches, String fieldName, Object expected, Object actual) {
            if (expected == null ? actual == null : expected.equals(actual)) {
                return;
            }

            if (!mismatches.isEmpty()) {
                mismatches.append(", ");
            }
            mismatches
                    .append(fieldName)
                    .append(" (expected ")
                    .append(summarizeValue(expected))
                    .append(", got ")
                    .append(summarizeValue(actual))
                    .append(")");
        }

        private static String summarizeValue(Object value) {
            String text = String.valueOf(value);
            return text.length() <= 80 ? text : text.substring(0, 77) + "...";
        }
    }

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final C2Config config;
    private final C2DeviceContext deviceContext;
    private final RawSubscriber observationSubscriber;
    private final Supplier<Pose3d> estimatedPoseSupplier;
    private final StructPublisher<Pose3d> estimatedPosePublisher;

    /**
     * Creates one IO source for all C2 cameras, publishing the current field-relative robot pose.
     */
    public VisionIOC2(
            CameraProperties[] cameraProperties,
            C2Config config,
            Supplier<Pose3d> estimatedPoseSupplier) {
        this.estimatedPoseSupplier =
                Objects.requireNonNull(
                        estimatedPoseSupplier, "estimatedPoseSupplier cannot be null");
        this.config = validateConfig(config);
        if (cameraProperties == null || cameraProperties.length == 0) {
            throw new IllegalArgumentException("At least one C2 camera is required");
        }
        if (java.util.Arrays.stream(cameraProperties).anyMatch(java.util.Objects::isNull)) {
            throw new IllegalArgumentException("C2 camera properties cannot contain null values");
        }
        this.deviceContext = getOrCreateDeviceContext(cameraProperties, this.config);

        estimatedPosePublisher =
                ntInstance
                        .getTable("/" + this.config.deviceId() + "/Feedback")
                        .getStructTopic("Pose", Pose3d.struct)
                        .publish();

        NetworkTable outputTable = ntInstance.getTable("/" + this.config.deviceId() + "/output");

        observationSubscriber =
                outputTable
                        .getRawTopic("observation")
                        .subscribe(
                                FLATBUFFER_TYPE,
                                new byte[0],
                                PubSubOption.sendAll(true),
                                PubSubOption.keepDuplicates(true),
                                PubSubOption.pollStorage(this.config.pollStorageDepth()));
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        publishConfigIfNeeded();
        estimatedPosePublisher.set(estimatedPoseSupplier.get());

        long nowUs = WPIUtilJNI.now();
        long lastChangeUs = observationSubscriber.getLastChange();

        inputs.connected =
                ntInstance.isConnected()
                        && observationSubscriber.exists()
                        && lastChangeUs > 0
                        && nowUs - lastChangeUs <= DISCONNECT_TIMEOUT_US;
        inputs.rawPacketType = NativePacketType.C2;

        TimestampedRaw[] unreadFrames = observationSubscriber.readQueue();
        if (unreadFrames.length == 0) {
            inputs.rawResults = new byte[0][];
            inputs.captureTimestampsUs = new long[0];
            inputs.publishTimestampsUs = new long[0];
            return;
        }

        ArrayList<byte[]> results = new ArrayList<>(unreadFrames.length);
        ArrayList<Long> captureTimestampsUs = new ArrayList<>(unreadFrames.length);
        ArrayList<Long> publishTimestampsUs = new ArrayList<>(unreadFrames.length);

        for (var unreadFrame : unreadFrames) {
            if (unreadFrame == null || unreadFrame.value == null || unreadFrame.value.length == 0) {
                continue;
            }
            results.add(unreadFrame.value);
            captureTimestampsUs.add(unreadFrame.timestamp);
            publishTimestampsUs.add(
                    unreadFrame.serverTime != 0 ? unreadFrame.serverTime : unreadFrame.timestamp);
        }

        inputs.rawResults = results.toArray(byte[][]::new);
        inputs.captureTimestampsUs = new long[captureTimestampsUs.size()];
        inputs.publishTimestampsUs = new long[publishTimestampsUs.size()];

        for (int i = 0; i < inputs.captureTimestampsUs.length; i++) {
            inputs.captureTimestampsUs[i] = captureTimestampsUs.get(i);
        }
        for (int i = 0; i < inputs.publishTimestampsUs.length; i++) {
            inputs.publishTimestampsUs[i] = publishTimestampsUs.get(i);
        }
    }

    private void publishConfigIfNeeded() {
        deviceContext.publishConfigIfNeeded();
    }

    private static C2Config validateConfig(C2Config config) {
        if (config == null) {
            throw new IllegalArgumentException("c2Config cannot be null");
        }
        if (config.deviceId() == null || config.deviceId().isBlank()) {
            throw new IllegalArgumentException("c2Config.deviceId cannot be blank");
        }
        if (config.tagLayout() == null) {
            throw new IllegalArgumentException("c2Config.tagLayout cannot be null");
        }
        if (config.tagLayoutJson() == null || config.tagLayoutJson().isBlank()) {
            throw new IllegalArgumentException("c2Config.tagLayoutJson cannot be blank");
        }
        if (config.pollStorageDepth() <= 0) {
            throw new IllegalArgumentException("c2Config.pollStorageDepth must be positive");
        }
        return config;
    }

    private static C2DeviceContext getOrCreateDeviceContext(
            CameraProperties[] cameraProperties, C2Config config) {
        return DEVICE_CONTEXTS.compute(
                config.deviceId(),
                (deviceId, existingContext) -> {
                    if (existingContext == null) {
                        return new C2DeviceContext(
                                NetworkTableInstance.getDefault(), cameraProperties, config);
                    }
                    existingContext.registerCameras(cameraProperties, config);
                    return existingContext;
                });
    }

    private static SharedDeviceConfig sharedConfigFor(
            CameraProperties[] cameraProperties, C2Config config) {
        return new SharedDeviceConfig(
                config.exposure(),
                config.gain(),
                cameraExtrinsicsFor(cameraProperties),
                config.fiducialSizeMeters(),
                config.tagLayoutJson());
    }

    private static List<Pose3d> cameraExtrinsicsFor(CameraProperties[] cameraProperties) {
        int maxIndex =
                Arrays.stream(cameraProperties)
                        .mapToInt(CameraProperties::index)
                        .max()
                        .orElseThrow();
        Pose3d[] extrinsics = new Pose3d[maxIndex + 1];
        for (CameraProperties properties : cameraProperties) {
            if (properties.index() < 0 || extrinsics[properties.index()] != null) {
                throw new IllegalArgumentException(
                        "C2 camera indices must be unique and non-negative");
            }
            Transform3d transform = properties.robotToCamera();
            extrinsics[properties.index()] =
                    new Pose3d(transform.getTranslation(), transform.getRotation());
        }
        for (int index = 0; index < extrinsics.length; index++) {
            if (extrinsics[index] == null) {
                throw new IllegalArgumentException("Missing C2 camera index " + index);
            }
        }
        return List.copyOf(Arrays.asList(extrinsics));
    }

    /**
     * Decodes raw C2 flatbuffer frames into camera or robot pose observations.
     *
     * <p>Each frame is decoded directly from the flatbuffer into the standardized {@link
     * CameraResult} type without an intermediate {@code PhotonPipelineResult}, keeping the C2 wire
     * format fully encapsulated in this IO layer.
     */
    public static CameraResult[] decodeResults(
            VisionIOInputs inputs, AprilTagFieldLayout tagLayout) {
        ArrayList<CameraResult> results = new ArrayList<>(inputs.rawResults.length);
        for (int i = 0; i < inputs.rawResults.length; i++) {
            byte[] raw = inputs.rawResults[i];
            if (raw == null || raw.length == 0) continue;

            long captureTs =
                    i < inputs.captureTimestampsUs.length ? inputs.captureTimestampsUs[i] : 0;
            long publishTs =
                    i < inputs.publishTimestampsUs.length ? inputs.publishTimestampsUs[i] : 0;

            CameraResult[] frameResults = decodeC2Frame(raw, captureTs, publishTs, tagLayout);
            if (frameResults != null) {
                results.addAll(java.util.Arrays.asList(frameResults));
            }
        }
        return results.toArray(CameraResult[]::new);
    }

    /**
     * Decodes one raw C2 flatbuffer frame into a {@link CameraResult}.
     *
     * <p>Combined C2 frames produce one robot-pose observation. Per-camera frames are retained for
     * compatibility with C2 configurations that publish individual camera solves.
     *
     * @return decoded result, or {@code null} if the byte array is not a valid flatbuffer
     */
    private static CameraResult[] decodeC2Frame(
            byte[] raw,
            long captureTimestampUs,
            long publishTimestampUs,
            AprilTagFieldLayout tagLayout) {
        if (tagLayout == null) {
            return null;
        }

        dsv0.Frame frame;
        try {
            frame = dsv0.Frame.getRootAsFrame(ByteBuffer.wrap(raw));
        } catch (RuntimeException e) {
            return null;
        }

        long resolvedPublish = publishTimestampUs != 0 ? publishTimestampUs : captureTimestampUs;
        switch (frame.resultsType()) {
            case dsv0.Results.PerCameraResults -> {
                dsv0.PerCameraResults perCamera =
                        (dsv0.PerCameraResults) frame.results(new dsv0.PerCameraResults());
                if (perCamera == null) return new CameraResult[0];
                ArrayList<CameraResult> cameraResults = new ArrayList<>(perCamera.resultsLength());
                for (int i = 0; i < perCamera.resultsLength(); i++) {
                    dsv0.CameraOutput cameraOutput = perCamera.results(i);
                    CameraResult cameraResult =
                            decodeCameraOutput(
                                    cameraOutput, tagLayout, captureTimestampUs, resolvedPublish);
                    if (cameraResult != null) cameraResults.add(cameraResult);
                }
                return cameraResults.toArray(CameraResult[]::new);
            }
            case dsv0.Results.CombinedResults -> {
                dsv0.CombinedResults combined =
                        (dsv0.CombinedResults) frame.results(new dsv0.CombinedResults());
                if (combined == null) return new CameraResult[0];
                dsv0.CameraObservation observation = combined.robotObservation();
                if (observation == null || observation.solution0() == null) {
                    return new CameraResult[0];
                }
                dsv0.PoseSolution solution = observation.solution0();
                int[] tagIds = new int[observation.tagIdsLength()];
                for (int i = 0; i < tagIds.length; i++) {
                    tagIds[i] = observation.tagIds(i);
                }
                return new CameraResult[] {
                    new CameraResult(
                            new TagObservation[0],
                            Optional.empty(),
                            (double) captureTimestampUs,
                            (double) resolvedPublish,
                            -1,
                            Optional.of(
                                    new RobotPoseObservation(
                                            tagIds,
                                            c2PoseToWpilib(solution),
                                            solution.reprojectionError())))
                };
            }
            default -> {
                return new CameraResult[0];
            }
        }
    }

    private static CameraResult decodeCameraOutput(
            dsv0.CameraOutput cameraOutput,
            AprilTagFieldLayout tagLayout,
            long captureTimestampUs,
            long publishTimestampUs) {
        if (cameraOutput == null) return null;
        int cameraIndex = cameraOutput.cameraIndex();
        dsv0.CameraObservation observation = cameraOutput.cameraObservation();
        if (observation == null || observation.solution0() == null) {
            return emptyC2Result(captureTimestampUs, publishTimestampUs, cameraIndex);
        }

        dsv0.PoseSolution primarySolution = observation.solution0();
        Pose3d fieldToCamera = c2PoseToWpilib(primarySolution);

        // Build one TagObservation for single-tag observations
        ArrayList<TagObservation> tagObs = new ArrayList<>(observation.tagIdsLength());
        if (observation.tagIdsLength() == 1) {
            int tagId = observation.tagIds(0);
            Optional<Pose3d> tagPoseOpt = tagLayout.getTagPose(tagId);
            if (tagPoseOpt.isPresent()) {
                // Use the primary solve as the best pose; alternate as the alt pose
                Optional<dsv0.PoseSolution> alternateSolution =
                        Optional.ofNullable(observation.solution1());
                Optional<Pose3d> alt = alternateSolution.map(VisionIOC2::c2PoseToWpilib);
                double ambiguity =
                        computeC2Ambiguity(
                                primarySolution.reprojectionError(),
                                alternateSolution
                                        .map(PoseSolution::reprojectionError)
                                        .orElse(-1.0));
                tagObs.add(new TagObservation(tagId, fieldToCamera, alt, 0.0, ambiguity));
            }
        }

        // Build a MultiTagObservation when ≥2 tags were used
        Optional<MultiTagObservation> multiTag = Optional.empty();
        if (observation.tagIdsLength() >= 2) {
            int[] ids = new int[observation.tagIdsLength()];
            for (int i = 0; i < observation.tagIdsLength(); i++) {
                ids[i] = observation.tagIds(i);
            }

            multiTag =
                    Optional.of(
                            new MultiTagObservation(
                                    ids, fieldToCamera, primarySolution.reprojectionError()));
        }

        return new CameraResult(
                tagObs.toArray(TagObservation[]::new),
                multiTag,
                (double) captureTimestampUs,
                (double) publishTimestampUs,
                cameraIndex);
    }

    private static CameraResult emptyC2Result(
            long captureTimestampUs, long publishTimestampUs, int cameraIndex) {
        return new CameraResult(
                new TagObservation[0],
                Optional.empty(),
                (double) captureTimestampUs,
                (double) publishTimestampUs,
                cameraIndex);
    }

    private static Pose3d c2PoseToWpilib(dsv0.PoseSolution solution) {
        dsv0.Pose3d pose = solution.pose();
        dsv0.Vec3 t = pose.translation();
        dsv0.Quaternion q = pose.rotation();
        return new Pose3d(
                new Translation3d(t.x(), t.y(), t.z()),
                new Rotation3d(
                        new edu.wpi.first.math.geometry.Quaternion(q.w(), q.x(), q.y(), q.z())));
    }

    private static double computeC2Ambiguity(double primaryError, double alternateError) {
        if (alternateError <= 0.0) return 0.0;
        return Math.max(0.0, Math.min(1.0, primaryError / alternateError));
    }
}
