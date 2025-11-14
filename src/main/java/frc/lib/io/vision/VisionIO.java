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

import java.util.Optional;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;

public interface VisionIO {
    public class VisionIOInputs implements LoggableInputs {
        public boolean connected = false;
        public Optional<Matrix<N3, N3>> cameraMatrix = Optional.empty();
        public Optional<Matrix<N8, N1>> distCoeffs = Optional.empty();
        public PhotonPipelineResult[] results = new PhotonPipelineResult[0];

        @Override
        public void toLog(LogTable table)
        {
            table.put("Connected", connected);
            table.put("CameraMatrixIsPresent", cameraMatrix.isPresent());
            if (cameraMatrix.isPresent()) {
                table.put("CameraMatrixData", cameraMatrix.get().getData());
            }
            table.put("DistCoeffsIsPresent", distCoeffs.isPresent());
            if (distCoeffs.isPresent()) {
                table.put("DistCoeffsData", distCoeffs.get().getData());
            }
            table.put("ResultsCount", results.length);
            for (int i = 0; i < results.length; i++) {
                table.put("Results/" + i, results[i]);
            }
        }

        @Override
        public void fromLog(LogTable table)
        {
            connected = table.get("Connected", false);

            cameraMatrix =
                table.get("CameraMatrixIsPresent", false)
                    ? Optional.of(
                        new Matrix<N3, N3>(
                            new SimpleMatrix(3, 3, true,
                                table.get("CameraMatrixData", new double[9]))))
                    : Optional.empty();

            distCoeffs =
                table.get("DistCoeffsIsPresent", false)
                    ? Optional.of(
                        new Matrix<N8, N1>(
                            new SimpleMatrix(8, 1, true,
                                table.get("DistCoeffsData", new double[8]))))
                    : Optional.empty();

            int count = table.get("ResultsCount", 0);
            results = new PhotonPipelineResult[count];
            for (int i = 0; i < count; i++) {
                results[i] = table.get("Results/" + i, new PhotonPipelineResult());
            }
        }

    }

    public default void updateInputs(VisionIOInputs inputs)
    {}
}
