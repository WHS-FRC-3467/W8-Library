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

package frc.lib.devices;

import java.util.function.BiConsumer;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import frc.lib.io.vision.VisionIO;
import frc.lib.io.vision.VisionIO.VisionIOInputs;

public class AprilTagCamera {
    public record CameraProperties(
        String name,
        Transform3d robotToCamera,
        Matrix<N3, N3> cameraMatrix,
        Matrix<N8, N1> distCoeffs,
        int resolutionWidth,
        int resolutionHeight,
        double stdDevFactor) {
    }

    private final CameraProperties properties;
    private final VisionIO io;
    private final VisionIOInputs inputs;

    private final BiConsumer<PhotonPipelineResult, CameraProperties> visionConsumer;

    public AprilTagCamera(
        CameraProperties properties,
        VisionIO io,
        BiConsumer<PhotonPipelineResult, CameraProperties> visionConsumer) {
        this.properties = properties;
        this.io = io;
        this.visionConsumer = visionConsumer;
        inputs = new VisionIOInputs(properties.cameraMatrix(), properties.distCoeffs());
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(properties.name(), inputs);

        if (!inputs.connected)
            return;

        Stream.of(inputs.results)
            .forEach((PhotonPipelineResult result) -> visionConsumer.accept(result, properties));
    }
}
