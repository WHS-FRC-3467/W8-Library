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

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {
    public class VisionIOInputs implements LoggableInputs {
        public boolean connected = false;
        public PhotonPipelineResult[] results = new PhotonPipelineResult[0];

        @Override
        public void toLog(LogTable table)
        {
            table.put("Connected", connected);
            table.put("ResultsCount", results.length);
            for (int i = 0; i < results.length; i++) {
                table.put("Results/" + i, results[i]);
            }
        }

        @Override
        public void fromLog(LogTable table)
        {
            connected = table.get("Connected", false);
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
