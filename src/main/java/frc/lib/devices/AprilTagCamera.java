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

import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.io.vision.VisionIO;
import frc.lib.io.vision.VisionIO.VisionIOInputs;
import lombok.Getter;

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

    private final VisionIO io;
    private final VisionIOInputs inputs;

    private final Alert mismatchedIntrinsicsAlert =
        new Alert(
            "Supplied intrinsics in code do not match intrinsics from replayed inputs! Defaulting to inputs!",
            AlertType.kWarning);

    @Getter
    private final CameraProperties properties;

    public AprilTagCamera(
        CameraProperties properties,
        VisionIO io)
    {
        this.io = io;
        inputs = new VisionIOInputs(properties.cameraMatrix(), properties.distCoeffs());

        // Get camera intrinsics from inputs to potentially pull from log if replaying
        Logger.processInputs(properties.name, inputs);

        Matrix<N3, N3> cameraMatrix = MatBuilder.fill(Nat.N3(), Nat.N3(), inputs.cameraMatrix);
        Matrix<N8, N1> distCoeffs = MatBuilder.fill(Nat.N8(), Nat.N1(), inputs.distCoeffs);

        if (!cameraMatrix.equals(properties.cameraMatrix)
            || !distCoeffs.equals(properties.distCoeffs)) {
            mismatchedIntrinsicsAlert.set(true);
        }

        this.properties =
            new CameraProperties(
                properties.name,
                properties.robotToCamera,
                cameraMatrix,
                distCoeffs,
                properties.resolutionWidth,
                properties.resolutionHeight,
                properties.stdDevFactor);
    }

    public Optional<PhotonPipelineResult[]> getUnreadResults()
    {
        io.updateInputs(inputs);
        Logger.processInputs(properties.name(), inputs);

        if (!inputs.connected)
            return Optional.empty();

        return Optional.of(inputs.results);
    }
}
