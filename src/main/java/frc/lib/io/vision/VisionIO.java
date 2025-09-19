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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.lib.util.Timestamped;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean connected = false;
        public PoseObservation[] poseObservations = new PoseObservation[0];
        public TagObservation[] allTargets = new TagObservation[0];
        public int[] tagIds = new int[0];
    }

    /** Represents a robot pose sample used for pose estimation. */
    public static record PoseObservation(
        Time timestamp,
        Pose3d pose,
        double ambiguity,
        int tagCount,
        Distance averageTagDistance) {
    }

    public static record TagObservation(
        int id,
        double ptich,
        double yaw,
        double area) {
    }

    public default void updateInputs(VisionIOInputs inputs,
        Timestamped<Rotation2d> timestampedHeading)
    {}
}
