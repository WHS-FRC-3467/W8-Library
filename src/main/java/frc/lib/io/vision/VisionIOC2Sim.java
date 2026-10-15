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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

import frc.lib.devices.AprilTagCamera.CameraProperties;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.DoubleFunction;

/**
 * {@link VisionIOC2} for simulation, against C2 running in its simulation mode ({@code python
 * src/main.py --sim} in C2, on this computer).
 *
 * <p>Reads results exactly like the real robot does. In addition, it gives C2 what it needs to
 * render the cameras:
 *
 * <ul>
 *   <li>{@code /<dev>/sim/true_pose} ({@code Pose3d}): every true-pose sample, each with its NT
 *       timestamp set to the sample's own FPGA time, so C2 can interpolate the pose at any capture
 *       instant.
 *   <li>{@code /<dev>/sim/robot_to_camera} ({@code Transform3d[]}): each camera's mount, indexed
 *       by C2 camera index.
 * </ul>
 */
public class VisionIOC2Sim extends VisionIOC2 {
    private static final ConcurrentHashMap<String, SimDevice> SIM_DEVICES =
            new ConcurrentHashMap<>();

    /** Simulation publishers shared by every camera on one C2 device. */
    private static final class SimDevice {
        private final DoubleFunction<NavigableMap<Double, Pose2d>> truePoseSamplesAfter;
        private final StructPublisher<Pose3d> truePosePublisher;
        private final StructArrayPublisher<Transform3d> mountsPublisher;
        private final Map<Integer, Transform3d> mountsByIndex = new TreeMap<>();
        private double lastPublishedSeconds = Double.NEGATIVE_INFINITY;

        private SimDevice(
                String deviceId, DoubleFunction<NavigableMap<Double, Pose2d>> truePoseSamplesAfter) {
            this.truePoseSamplesAfter = truePoseSamplesAfter;
            NetworkTable sim = NetworkTableInstance.getDefault().getTable("/" + deviceId + "/sim");
            truePosePublisher =
                    sim.getStructTopic("true_pose", Pose3d.struct)
                            .publish(
                                    PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
            mountsPublisher =
                    sim.getStructArrayTopic("robot_to_camera", Transform3d.struct).publish();
        }

        synchronized void addMount(int cameraIndex, Transform3d robotToCamera) {
            mountsByIndex.put(cameraIndex, robotToCamera);
            int count = mountsByIndex.keySet().stream().mapToInt(i -> i + 1).max().orElse(0);
            Transform3d[] mounts = new Transform3d[count];
            for (int i = 0; i < count; i++) {
                mounts[i] = mountsByIndex.getOrDefault(i, Transform3d.kZero);
            }
            mountsPublisher.set(mounts);
        }

        /** Publishes the true-pose samples not sent yet. Cheap to call once per camera per loop. */
        synchronized void publishTruePose() {
            for (Map.Entry<Double, Pose2d> sample :
                    truePoseSamplesAfter.apply(lastPublishedSeconds).entrySet()) {
                truePosePublisher.set(
                        new Pose3d(sample.getValue()), (long) (sample.getKey() * 1e6));
                lastPublishedSeconds = sample.getKey();
            }
        }
    }

    private final SimDevice simDevice;

    /**
     * Creates the simulated IO for one C2 camera.
     *
     * @param properties camera name, resolution, mount, etc.
     * @param config C2 device and topic configuration
     * @param truePoseSamplesAfter returns true-pose samples newer than the given FPGA time
     *     (seconds), keyed by sample time. In simulation, odometry has no wheel slip, so the
     *     odometry pose is the true pose.
     */
    public VisionIOC2Sim(
            CameraProperties properties,
            C2Config config,
            DoubleFunction<NavigableMap<Double, Pose2d>> truePoseSamplesAfter) {
        super(properties, config);
        simDevice =
                SIM_DEVICES.computeIfAbsent(
                        config.deviceId(), id -> new SimDevice(id, truePoseSamplesAfter));
        simDevice.addMount(config.cameraIndex(), properties.robotToCamera());
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        simDevice.publishTruePose();
        super.updateInputs(inputs);
    }
}
