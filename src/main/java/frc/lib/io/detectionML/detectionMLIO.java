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

package frc.lib.io.detectionML;

import org.littletonrobotics.junction.AutoLog;

/**
 * Standardized interface for ML-IO used in FRC.
 */
public interface detectionMLIO {

    /* Abstract class effectively behaving as a visible structure for data encapsulation. */
    @AutoLog
    abstract class detectionMLIOInputs {
        /** Whether the camera is connected. */
        public boolean connected = false;
        /** ID of detected object. */
        public int objID = -1000;
        /** Confidence of detected object. */
        public float objConf = -1000.0f;
        /** Circumscribed area of detected object. */
        public double objArea = -1000.0;
        /** Pitch of detected object. */
        public double pitch = -1000.0;
        /** Yaw of detected object. */
        public double yaw = -1000.0;
        /** Skew of detected object. */
        public double skew = -1000.0;
    }

    /*
     * Name of the camera capturing optical data.
     */
    public default String cameraString()
    {
        return "";
    }

    /*
     * Updates the provided {@link detectionMLIOInputs} instance using the latest camera readings.
     * If the camera is not connected, it populates the field with default values.
     * 
     * @param inputs The structure to populate with updated camera readings.
     */
    public default void updateInputs(detectionMLIOInputs inputs)
    {}
}
