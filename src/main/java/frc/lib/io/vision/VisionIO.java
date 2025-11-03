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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean connected = false;
        public VisionObservation[] poseObservations = null;
    }

    public record Camera(String name, Transform3d robotToCamera, Matrix<N3, N3> cameraMatrix,
        Matrix<N8, N1> distCoeffs) {
    }

    public static record TagObservation(
        int id,
        double area,
        Angle pitch,
        Angle yaw,
        Distance distance) {
    }

    /** Represents a robot pose sample used for pose estimation. */
    public static record VisionObservation(
        Time timestamp,
        Camera camera,
        PhotonPipelineResult photonResult,
        Transform3d bestCameraToTarget,
        double ambiguity,
        List<TagObservation> tagObservations) {
    }

    public default void updateInputs(VisionIOInputs inputs)
    {}
}
