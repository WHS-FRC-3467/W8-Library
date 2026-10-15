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

import frc.lib.devices.VisionSource.CameraProperties;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.NavigableMap;
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
 *   <li>{@code /<dev>/sim/robot_to_camera} ({@code Transform3d[]}): each camera's mount, indexed by
 *       its slice of C2's combined frame.
 * </ul>
 */
public class VisionIOC2Sim extends VisionIOC2 {
    private final DoubleFunction<NavigableMap<Double, Pose2d>> truePoseSamplesAfter;
    private final StructPublisher<Pose3d> truePosePublisher;
    // Held so the topic stays published.
    private final StructArrayPublisher<Transform3d> mountsPublisher;
    private double lastPublishedSeconds = Double.NEGATIVE_INFINITY;

    /**
     * @param config the device's configuration
     * @param cameras the cameras of this source, in the same order as {@code config.cameraSlices()}
     * @param truePoseSamplesAfter returns true-pose samples newer than the given FPGA time
     *     (seconds), keyed by sample time. In simulation, odometry has no wheel slip, so the
     *     odometry pose is the true pose.
     */
    public VisionIOC2Sim(
            C2Config config,
            List<CameraProperties> cameras,
            DoubleFunction<NavigableMap<Double, Pose2d>> truePoseSamplesAfter) {
        super(config);
        if (cameras.size() != config.cameraSlices().size()) {
            throw new IllegalArgumentException(
                    "VisionIOC2Sim needs one camera per C2 slice: "
                            + cameras.size()
                            + " cameras, slices "
                            + config.cameraSlices());
        }
        this.truePoseSamplesAfter = truePoseSamplesAfter;
        NetworkTable sim =
                NetworkTableInstance.getDefault().getTable("/" + config.deviceId() + "/sim");
        truePosePublisher =
                sim.getStructTopic("true_pose", Pose3d.struct)
                        .publish(PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
        mountsPublisher = sim.getStructArrayTopic("robot_to_camera", Transform3d.struct).publish();
        mountsPublisher.set(mountsBySlice(config.cameraSlices(), cameras));
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        for (Map.Entry<Double, Pose2d> sample :
                truePoseSamplesAfter.apply(lastPublishedSeconds).entrySet()) {
            truePosePublisher.set(new Pose3d(sample.getValue()), (long) (sample.getKey() * 1e6));
            lastPublishedSeconds = sample.getKey();
        }
        super.updateInputs(inputs);
    }

    private static Transform3d[] mountsBySlice(
            List<Integer> slices, List<CameraProperties> cameras) {
        int count = slices.stream().mapToInt(slice -> slice + 1).max().orElse(0);
        Transform3d[] mounts = new Transform3d[count];
        Arrays.fill(mounts, Transform3d.kZero);
        for (int i = 0; i < slices.size(); i++) {
            mounts[slices.get(i)] = cameras.get(i).robotToCamera();
        }
        return mounts;
    }
}
