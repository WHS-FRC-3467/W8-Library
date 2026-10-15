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

import frc.robot.FieldConstants.AprilTagLayoutType;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

/**
 * {@link VisionIO} for one C2 coprocessor: one source serving all of its cameras.
 *
 * <p>NetworkTables layout used (see C2's README for the full list), for device {@code dsv0}:
 *
 * <ul>
 *   <li>{@code /dsv0/config/*}: capture settings (camera id, per-camera resolution, exposure,
 *       gain).
 *   <li>{@code /dsv0/pipelines/apriltag/enabled} and {@code .../config/*}: the AprilTag pipeline's
 *       settings (tag layout, tag size, minimum tag size in pixels).
 *   <li>{@code /dsv0/pipelines/apriltag/output/camera_<slice>}: one flatbuffer per frame per
 *       camera, including frames with no tags (decoded by {@link C2Decoder}).
 *   <li>{@code /dsv0/status/heartbeat} and {@code /dsv0/status/camera_<slice>/connected}: health.
 * </ul>
 *
 * <p>The robot is the NT server, so the settings published here persist on the server and C2 picks
 * them up whenever it (re)connects. C2 only sets defaults on these topics, so ours always win.
 *
 * <p>Timestamps: C2 publishes each result with the NT timestamp set to the frame's capture time on
 * the robot's clock (FPGA time), corrected for clock drift. No extra offset is needed.
 */
public class VisionIOC2 implements VisionIO {
    public static final String DEFAULT_DEVICE_ID = "dsv0";
    private static final String PIPELINE = "apriltag";
    private static final String FLATBUFFER_TYPE = "dsv0_fb";
    /** C2's heartbeat ticks every 100 ms; this long without one means it is gone. */
    private static final long HEARTBEAT_TIMEOUT_US = 500_000L;

    /**
     * Configuration for one C2 device.
     *
     * @param deviceId NetworkTables root of the C2 device, e.g. {@code dsv0}
     * @param cameraSlices for each camera of this source (in the source's camera order), which
     *     slice of C2's combined frame it is
     * @param cameraId capture device consumed by C2
     * @param resolutionWidth width of ONE camera, pixels
     * @param resolutionHeight camera height, pixels
     * @param exposure exposure sent to C2
     * @param gain gain sent to C2
     * @param fiducialSizeMeters AprilTag edge length used by C2's solvePnP
     * @param tagLayoutJson serialized field layout for C2
     * @param minTagSizePx tags smaller than this in the image are left out of C2's solve; 0 = off
     * @param pollStorageDepth NT queue depth for unread results, per camera
     */
    public static record C2Config(
            String deviceId,
            List<Integer> cameraSlices,
            String cameraId,
            int resolutionWidth,
            int resolutionHeight,
            int exposure,
            int gain,
            double fiducialSizeMeters,
            String tagLayoutJson,
            double minTagSizePx,
            int pollStorageDepth) {

        /**
         * Defaults for device {@code dsv0}.
         *
         * @param resolutionWidth width of one camera, pixels
         * @param resolutionHeight camera height, pixels
         * @param cameraSlices each camera's slice of the combined frame, in the source's order
         * @return the configuration
         */
        public static C2Config defaults(
                int resolutionWidth, int resolutionHeight, List<Integer> cameraSlices) {
            return new C2Config(
                    DEFAULT_DEVICE_ID,
                    cameraSlices,
                    "0",
                    resolutionWidth,
                    resolutionHeight,
                    100,
                    0,
                    Units.inchesToMeters(6.5),
                    AprilTagLayoutType.NO_TRENCH.getLayoutString(),
                    // Tags smaller than this have noisy corners. In C2's simulation, 50 px cut the
                    // worst multi-tag error from 76 cm to under 7 cm. With these cameras' focal
                    // lengths (~1380-2000 px), 50 px is a tag about 4.6-6.6 m away.
                    50.0,
                    32);
        }

        void validate() {
            if (deviceId == null || deviceId.isBlank()) {
                throw new IllegalArgumentException("C2Config.deviceId cannot be blank");
            }
            if (cameraSlices == null || cameraSlices.isEmpty()) {
                throw new IllegalArgumentException("C2Config.cameraSlices cannot be empty");
            }
            if (cameraSlices.stream().anyMatch(slice -> slice == null || slice < 0)
                    || new HashSet<>(cameraSlices).size() != cameraSlices.size()) {
                throw new IllegalArgumentException(
                        "C2Config.cameraSlices must be distinct and non-negative: " + cameraSlices);
            }
            if (cameraId == null) {
                throw new IllegalArgumentException("C2Config.cameraId cannot be null");
            }
            if (tagLayoutJson == null || tagLayoutJson.isBlank()) {
                throw new IllegalArgumentException("C2Config.tagLayoutJson cannot be blank");
            }
            if (minTagSizePx < 0) {
                throw new IllegalArgumentException("C2Config.minTagSizePx cannot be negative");
            }
            if (pollStorageDepth <= 0) {
                throw new IllegalArgumentException("C2Config.pollStorageDepth must be positive");
            }
        }
    }

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final IntegerSubscriber heartbeat;
    private final List<BooleanSubscriber> camerasConnected = new ArrayList<>();
    private final List<RawSubscriber> results = new ArrayList<>();

    // Held so the topics stay published.
    private final List<Object> publishers = new ArrayList<>();

    /**
     * @param config the device's configuration
     */
    public VisionIOC2(C2Config config) {
        config.validate();
        NetworkTable device = ntInstance.getTable("/" + config.deviceId());
        publishSettings(device, config);

        NetworkTable status = device.getSubTable("status");
        heartbeat = status.getIntegerTopic("heartbeat").subscribe(0);
        NetworkTable outputs =
                device.getSubTable("pipelines").getSubTable(PIPELINE).getSubTable("output");
        for (int slice : config.cameraSlices()) {
            String camera = "camera_" + slice;
            camerasConnected.add(
                    status.getSubTable(camera).getBooleanTopic("connected").subscribe(false));
            results.add(
                    outputs.getRawTopic(camera)
                            .subscribe(
                                    FLATBUFFER_TYPE,
                                    new byte[0],
                                    PubSubOption.sendAll(true),
                                    PubSubOption.keepDuplicates(true),
                                    PubSubOption.pollStorage(config.pollStorageDepth())));
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.rawPacketType = NativePacketType.C2;
        inputs.reportsSourceHealth = true;
        long nowUs = WPIUtilJNI.now();
        long lastBeatUs = heartbeat.getLastChange();
        inputs.connected =
                ntInstance.isConnected()
                        && lastBeatUs > 0
                        && nowUs - lastBeatUs <= HEARTBEAT_TIMEOUT_US;

        int cameras = results.size();
        inputs.camerasConnected = new boolean[cameras];
        ArrayList<TimestampedRaw> unread = new ArrayList<>();
        ArrayList<Integer> resultCameras = new ArrayList<>();
        for (int i = 0; i < cameras; i++) {
            // C2 reports each camera separately (images arriving and not blank).
            inputs.camerasConnected[i] = inputs.connected && camerasConnected.get(i).get();
            // Every result since the last loop, oldest first. The caller decides how many to use.
            for (TimestampedRaw result : results.get(i).readQueue()) {
                if (result != null && result.value != null && result.value.length > 0) {
                    unread.add(result);
                    resultCameras.add(i);
                }
            }
        }

        inputs.rawResults = new byte[unread.size()][];
        inputs.resultCameras = resultCameras.stream().mapToInt(Integer::intValue).toArray();
        inputs.captureTimestampsUs = new long[unread.size()];
        inputs.publishTimestampsUs = new long[unread.size()];
        for (int i = 0; i < unread.size(); i++) {
            inputs.rawResults[i] = unread.get(i).value;
            // The NT timestamp IS the capture time, on the robot's clock (see class docs).
            inputs.captureTimestampsUs[i] = unread.get(i).timestamp;
            // NT doesn't record arrival time; this is when the robot read it (<= one loop later).
            inputs.publishTimestampsUs[i] = nowUs;
        }
    }

    private void publishSettings(NetworkTable device, C2Config config) {
        NetworkTable capture = device.getSubTable("config");
        StringPublisher cameraId = capture.getStringTopic("camera_id").publish();
        IntegerPublisher width = capture.getIntegerTopic("camera_resolution_width").publish();
        IntegerPublisher height = capture.getIntegerTopic("camera_resolution_height").publish();
        IntegerPublisher exposure = capture.getIntegerTopic("camera_exposure").publish();
        IntegerPublisher gain = capture.getIntegerTopic("camera_gain").publish();

        NetworkTable pipeline = device.getSubTable("pipelines").getSubTable(PIPELINE);
        BooleanPublisher enabled = pipeline.getBooleanTopic("enabled").publish();
        NetworkTable pipelineConfig = pipeline.getSubTable("config");
        StringPublisher tagLayout = pipelineConfig.getStringTopic("tag_layout").publish();
        DoublePublisher tagSize = pipelineConfig.getDoubleTopic("fiducial_size_m").publish();
        DoublePublisher minTagSize = pipelineConfig.getDoubleTopic("min_tag_size_px").publish();

        cameraId.set(config.cameraId());
        width.set(config.resolutionWidth());
        height.set(config.resolutionHeight());
        exposure.set(config.exposure());
        gain.set(config.gain());
        enabled.set(true);
        tagLayout.set(config.tagLayoutJson());
        tagSize.set(config.fiducialSizeMeters());
        minTagSize.set(config.minTagSizePx());
        publishers.addAll(
                List.of(
                        cameraId, width, height, exposure, gain, enabled, tagLayout, tagSize,
                        minTagSize));
    }
}
