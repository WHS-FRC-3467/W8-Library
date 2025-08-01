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

package frc.lib.io.DetectionML;

import org.littletonrobotics.junction.AutoLog;
import java.util.List;

/**
 * Standardized interface for ML-IO used in FRC.
 */
public interface DetectionMLIO {

    /*
     * Abstract class defining data type for updateInputs method.
     */
    @AutoLog
    abstract class DetectionMLIOInputs {
        /** Whether the camera is connected. */
        public boolean connected = false;
        /** Data structure (via record) containing target information. */
        public List<TargetObservation> latestTargetObservation;
    }

    /* Data structure of target information. */
    public static record TargetObservation(
        /** ID of detected object. */
        int objID,
        /** Confidence of detected object. */
        float objConf,
        /** Circumscribed area of detected object. */
        double objArea,
        /** Pitch of detected object. */
        double pitch,
        /** Yaw of detected object. */
        double yaw,
        /** Skew of detected object. */
        double skew) {
    }

    /*
     * Name of the camera capturing optical data.
     */
    public default String getCamera()
    {
        return "";
    }

    /*
     * Updates the provided {@link detectionMLIOInputs} instance using the latest camera readings.
     * If the camera is not connected, DetectionMLIOInput fields remain empty.
     * 
     * @param inputs The structure to populate with updated camera readings.
     */
    public default void updateInputs(DetectionMLIOInputs inputs)
    {}
}
