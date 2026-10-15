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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.TimestampedRaw;
import edu.wpi.first.util.WPIUtilJNI;

import frc.lib.devices.AprilTagCamera.CameraProperties;
import frc.robot.FieldConstants.AprilTagLayoutType;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;

/**
 * {@link VisionIO} for one camera of a C2 multi-camera coprocessor.
 *
 * <p>NetworkTables layout used (see C2's README for the full list), for device {@code dsv0}:
 *
 * <ul>
 *   <li>{@code /dsv0/config/*}: capture settings (camera id, per-camera resolution, exposure,
 *       gain), shared by every camera on the device.
 *   <li>{@code /dsv0/pipelines/apriltag/enabled}, {@code .../config/tag_layout}, {@code
 *       .../config/fiducial_size_m}, {@code .../config/min_tag_size_px}: the AprilTag pipeline's
 *       settings.
 *   <li>{@code /dsv0/pipelines/apriltag/output/camera_<i>}: one flatbuffer ({@code dsv0.Frame})
 *       per frame for camera {@code i}, including frames with no tags.
 *   <li>{@code /dsv0/status/heartbeat} and {@code /dsv0/status/camera_<i>/connected}: health.
 * </ul>
 *
 * <p>Timestamps: C2 publishes each result with the NT timestamp set to the frame's capture time on
 * the robot's clock (FPGA time), corrected for clock drift. {@link TimestampedRaw#timestamp} is
 * therefore directly comparable to {@code Timer.getFPGATimestamp()} (in microseconds), in real
 * matches and in simulation. No extra offset should be needed.
 */
public class VisionIOC2 implements VisionIO {
    private static final String DEFAULT_DEVICE_ID = "dsv0";
    private static final String DEFAULT_CAMERA_ID = "0";
    private static final String PIPELINE = "apriltag";
    private static final String FLATBUFFER_TYPE = "dsv0_fb";
    private static final int DEFAULT_CAMERA_EXPOSURE = 100;
    private static final int DEFAULT_CAMERA_GAIN = 0;
    private static final int DEFAULT_POLL_STORAGE_DEPTH = 32;
    /** C2's heartbeat ticks every 100 ms; this many microseconds without one means it is gone. */
    private static final long HEARTBEAT_TIMEOUT_US = 500_000L;

    private static final double DEFAULT_FIDUCIAL_SIZE_METERS = Units.inchesToMeters(6.5);

    /**
     * Tags smaller than this (longest edge, pixels) are left out of C2's solve: tiny tags have
     * noisy corners. In C2's simulation, 50 px cut the worst multi-tag error from 76 cm to under 7
     * cm. With these cameras' focal lengths (~1380-2000 px), 50 px is a tag about 4.6-6.6 m away.
     */
    private static final double DEFAULT_MIN_TAG_SIZE_PX = 50.0;
    private static final ConcurrentHashMap<String, C2Device> DEVICES = new ConcurrentHashMap<>();

    /**
     * Configuration for one C2 camera.
     *
     * @param deviceId NetworkTables root of the C2 device, e.g. {@code dsv0}
     * @param cameraIndex This camera's slice of the module's combined frame
     * @param cameraId Capture device consumed by C2 (shared by the device)
     * @param exposure Exposure sent to C2 (shared by the device)
     * @param gain Gain sent to C2 (shared by the device)
     * @param fiducialSizeMeters AprilTag edge length used by C2's solvePnP
     * @param tagLayoutJson Serialized {@link AprilTagFieldLayout} for C2
     * @param minTagSizePx Tags smaller than this in the image are left out of C2's solve; 0 = off
     * @param pollStorageDepth NT queue depth for unread results
     */
    public static record C2Config(
            String deviceId,
            int cameraIndex,
            String cameraId,
            int exposure,
            int gain,
            double fiducialSizeMeters,
            String tagLayoutJson,
            double minTagSizePx,
            int pollStorageDepth) {}

    /** Settings every camera on the same C2 device must agree on. */
    private static record SharedConfig(
            String cameraId,
            int resolutionWidth,
            int resolutionHeight,
            int exposure,
            int gain,
            double fiducialSizeMeters,
            String tagLayoutJson,
            double minTagSizePx) {}

    /**
     * Publishers and subscribers shared by every camera on one C2 device.
     *
     * <p>The robot is the NT server, so values published here persist on the server and C2 picks
     * them up whenever it (re)connects. C2 only sets defaults on these topics, so ours always win.
     */
    private static final class C2Device {
        private final String deviceId;
        private final Map<Integer, String> cameraNamesByIndex = new HashMap<>();
        private final NetworkTable table;
        private final IntegerSubscriber heartbeat;

        private SharedConfig sharedConfig = null;

        // Held so the topics stay published.
        private final StringPublisher cameraIdPublisher;
        private final IntegerPublisher widthPublisher;
        private final IntegerPublisher heightPublisher;
        private final IntegerPublisher exposurePublisher;
        private final IntegerPublisher gainPublisher;
        private final BooleanPublisher pipelineEnabledPublisher;
        private final StringPublisher tagLayoutPublisher;
        private final DoublePublisher fiducialSizePublisher;
        private final DoublePublisher minTagSizePublisher;

        private C2Device(NetworkTableInstance ntInstance, String deviceId) {
            this.deviceId = deviceId;
            table = ntInstance.getTable("/" + deviceId);

            NetworkTable capture = table.getSubTable("config");
            cameraIdPublisher = capture.getStringTopic("camera_id").publish();
            widthPublisher = capture.getIntegerTopic("camera_resolution_width").publish();
            heightPublisher = capture.getIntegerTopic("camera_resolution_height").publish();
            exposurePublisher = capture.getIntegerTopic("camera_exposure").publish();
            gainPublisher = capture.getIntegerTopic("camera_gain").publish();

            NetworkTable pipeline = table.getSubTable("pipelines").getSubTable(PIPELINE);
            pipelineEnabledPublisher = pipeline.getBooleanTopic("enabled").publish();
            NetworkTable pipelineConfig = pipeline.getSubTable("config");
            tagLayoutPublisher = pipelineConfig.getStringTopic("tag_layout").publish();
            fiducialSizePublisher = pipelineConfig.getDoubleTopic("fiducial_size_m").publish();
            minTagSizePublisher = pipelineConfig.getDoubleTopic("min_tag_size_px").publish();

            heartbeat = table.getSubTable("status").getIntegerTopic("heartbeat").subscribe(0);
        }

        synchronized void registerCamera(CameraProperties properties, C2Config config) {
            String existing =
                    cameraNamesByIndex.putIfAbsent(config.cameraIndex(), properties.name());
            if (existing != null && !existing.equals(properties.name())) {
                throw new IllegalArgumentException(
                        "Cameras \""
                                + existing
                                + "\" and \""
                                + properties.name()
                                + "\" both use C2 camera index "
                                + config.cameraIndex()
                                + " on device \""
                                + deviceId
                                + "\".");
            }

            SharedConfig candidate =
                    new SharedConfig(
                            config.cameraId(),
                            properties.resolutionWidth(),
                            properties.resolutionHeight(),
                            config.exposure(),
                            config.gain(),
                            config.fiducialSizeMeters(),
                            config.tagLayoutJson(),
                            config.minTagSizePx());
            if (sharedConfig == null) {
                sharedConfig = candidate;
                publish(candidate);
            } else if (!sharedConfig.equals(candidate)) {
                throw new IllegalArgumentException(
                        "Camera \""
                                + properties.name()
                                + "\" disagrees with the other cameras on C2 device \""
                                + deviceId
                                + "\" about device-wide settings (camera id, resolution, exposure,"
                                + " gain, tag size, tag layout, or min tag size): "
                                + sharedConfig
                                + " vs "
                                + candidate);
            }
        }

        private void publish(SharedConfig config) {
            cameraIdPublisher.set(config.cameraId());
            widthPublisher.set(config.resolutionWidth());
            heightPublisher.set(config.resolutionHeight());
            exposurePublisher.set(config.exposure());
            gainPublisher.set(config.gain());
            pipelineEnabledPublisher.set(true);
            tagLayoutPublisher.set(config.tagLayoutJson());
            fiducialSizePublisher.set(config.fiducialSizeMeters());
            minTagSizePublisher.set(config.minTagSizePx());
        }

        /** Whether C2 is running and connected (its heartbeat is fresh). */
        boolean isAlive(long nowUs) {
            long lastBeatUs = heartbeat.getLastChange();
            return lastBeatUs > 0 && nowUs - lastBeatUs <= HEARTBEAT_TIMEOUT_US;
        }

        NetworkTable table() {
            return table;
        }
    }

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final C2Device device;
    private final BooleanSubscriber cameraConnected;
    private final RawSubscriber results;

    /**
     * Creates the IO for one C2 camera.
     *
     * @param properties camera name, resolution, etc.
     * @param config C2 device and topic configuration
     */
    public VisionIOC2(CameraProperties properties, C2Config config) {
        validate(config);
        device =
                DEVICES.computeIfAbsent(
                        config.deviceId(), id -> new C2Device(NetworkTableInstance.getDefault(), id));
        device.registerCamera(properties, config);

        String camera = "camera_" + config.cameraIndex();
        cameraConnected =
                device.table()
                        .getSubTable("status")
                        .getSubTable(camera)
                        .getBooleanTopic("connected")
                        .subscribe(false);
        results =
                device.table()
                        .getSubTable("pipelines")
                        .getSubTable(PIPELINE)
                        .getSubTable("output")
                        .getRawTopic(camera)
                        .subscribe(
                                FLATBUFFER_TYPE,
                                new byte[0],
                                PubSubOption.sendAll(true),
                                PubSubOption.keepDuplicates(true),
                                PubSubOption.pollStorage(config.pollStorageDepth()));
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        long nowUs = WPIUtilJNI.now();
        inputs.connected =
                ntInstance.isConnected() && device.isAlive(nowUs) && cameraConnected.get();
        inputs.rawPacketType = NativePacketType.C2;

        // Every result since the last loop, oldest first. The caller decides how many to use.
        TimestampedRaw[] unread = results.readQueue();
        ArrayList<TimestampedRaw> valid = new ArrayList<>(unread.length);
        for (TimestampedRaw result : unread) {
            if (result != null && result.value != null && result.value.length > 0) {
                valid.add(result);
            }
        }

        inputs.rawResults = new byte[valid.size()][];
        inputs.captureTimestampsUs = new long[valid.size()];
        inputs.publishTimestampsUs = new long[valid.size()];
        for (int i = 0; i < valid.size(); i++) {
            inputs.rawResults[i] = valid.get(i).value;
            // The NT timestamp IS the capture time, on the robot's clock (see class docs).
            inputs.captureTimestampsUs[i] = valid.get(i).timestamp;
            // NT doesn't record arrival time; this is when the robot read it (<= one loop later).
            inputs.publishTimestampsUs[i] = nowUs;
        }
    }

    /**
     * Default configuration for camera {@code cameraIndex} of device {@code dsv0}.
     *
     * @param cameraIndex the camera's slice of the module's combined frame
     * @return the configuration
     */
    public static C2Config defaultsFor(int cameraIndex) {
        return new C2Config(
                DEFAULT_DEVICE_ID,
                cameraIndex,
                DEFAULT_CAMERA_ID,
                DEFAULT_CAMERA_EXPOSURE,
                DEFAULT_CAMERA_GAIN,
                DEFAULT_FIDUCIAL_SIZE_METERS,
                AprilTagLayoutType.NO_TRENCH.getLayoutString(),
                DEFAULT_MIN_TAG_SIZE_PX,
                DEFAULT_POLL_STORAGE_DEPTH);
    }

    private static void validate(C2Config config) {
        if (config == null) {
            throw new IllegalArgumentException("C2Config cannot be null");
        }
        if (config.deviceId() == null || config.deviceId().isBlank()) {
            throw new IllegalArgumentException("C2Config.deviceId cannot be blank");
        }
        if (config.cameraIndex() < 0) {
            throw new IllegalArgumentException("C2Config.cameraIndex must be non-negative");
        }
        if (config.cameraId() == null) {
            throw new IllegalArgumentException("C2Config.cameraId cannot be null");
        }
        if (config.tagLayoutJson() == null || config.tagLayoutJson().isBlank()) {
            throw new IllegalArgumentException("C2Config.tagLayoutJson cannot be blank");
        }
        if (config.minTagSizePx() < 0) {
            throw new IllegalArgumentException("C2Config.minTagSizePx cannot be negative");
        }
        if (config.pollStorageDepth() <= 0) {
            throw new IllegalArgumentException("C2Config.pollStorageDepth must be positive");
        }
    }

    /**
     * Decodes the raw C2 results stored in {@code inputs} (possibly from a replayed log) into
     * {@link CameraResult}s.
     *
     * @param inputs the logged inputs
     * @param tagLayout field layout, to look up observed tags
     * @param cameraIndex which camera's output to extract
     * @return one result per decodable frame
     */
    public static CameraResult[] decodeResults(
            VisionIOInputs inputs, AprilTagFieldLayout tagLayout, int cameraIndex) {
        ArrayList<CameraResult> decoded = new ArrayList<>(inputs.rawResults.length);
        for (int i = 0; i < inputs.rawResults.length; i++) {
            byte[] raw = inputs.rawResults[i];
            if (raw == null || raw.length == 0 || i >= inputs.captureTimestampsUs.length) {
                continue;
            }
            long publishTimestampUs =
                    i < inputs.publishTimestampsUs.length
                            ? inputs.publishTimestampsUs[i]
                            : inputs.captureTimestampsUs[i];
            CameraResult result =
                    decodeFrame(
                            raw,
                            inputs.captureTimestampsUs[i],
                            publishTimestampUs,
                            tagLayout,
                            cameraIndex);
            if (result != null) {
                decoded.add(result);
            }
        }
        return decoded.toArray(CameraResult[]::new);
    }

    /**
     * Decodes one {@code dsv0.Frame}.
     *
     * @return the result (empty if no tags were seen), or {@code null} if {@code raw} is not a
     *     valid frame
     */
    private static CameraResult decodeFrame(
            byte[] raw,
            long captureTimestampUs,
            long publishTimestampUs,
            AprilTagFieldLayout tagLayout,
            int cameraIndex) {
        if (tagLayout == null || cameraIndex < 0) {
            return null;
        }

        dsv0.Frame frame;
        try {
            frame = dsv0.Frame.getRootAsFrame(ByteBuffer.wrap(raw));
        } catch (RuntimeException e) {
            return null;
        }

        dsv0.CameraOutput cameraOutput = findCameraOutput(frame, cameraIndex);
        dsv0.CameraObservation observation =
                cameraOutput == null ? null : cameraOutput.cameraObservation();
        if (observation == null || observation.solution0() == null) {
            return emptyResult(captureTimestampUs, publishTimestampUs);
        }

        dsv0.PoseSolution best = observation.solution0();
        Pose3d fieldToCamera = toWpilibPose(best);

        // Single tag: two candidate poses (IPPE); report both with an ambiguity ratio.
        ArrayList<TagObservation> tagObservations = new ArrayList<>(1);
        if (observation.tagIdsLength() == 1) {
            int tagId = observation.tagIds(0);
            if (tagLayout.getTagPose(tagId).isPresent()) {
                Optional<dsv0.PoseSolution> alternate = Optional.ofNullable(observation.solution1());
                tagObservations.add(
                        new TagObservation(
                                tagId,
                                fieldToCamera,
                                alternate.map(VisionIOC2::toWpilibPose),
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

    private static dsv0.CameraOutput findCameraOutput(dsv0.Frame frame, int cameraIndex) {
        for (int i = 0; i < frame.camerasLength(); i++) {
            dsv0.CameraOutput candidate = frame.cameras(i);
            if (candidate != null && candidate.cameraIndex() == cameraIndex) {
                return candidate;
            }
        }
        return null;
    }

    private static CameraResult emptyResult(long captureTimestampUs, long publishTimestampUs) {
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
