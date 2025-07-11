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
@AutoLog
public interface detection_ML_IO {
    /*
     * Name of the camera capturing optical data.
     */
    public default String cameraString()
    {
        return "";
    }

    /*
     * Update observations from camera.
     */
    public default void updateInputs()
    {

    }

    /*
     * Returns pitch of detected object.
     */
    public default double returnPitch()
    {
        return -1000.0;
    }

    /*
     * Returns yaw of detected object.
     */
    public default double returnYaw()
    {
        return -1000.0;
    }

    /*
     * Returns skew of detected object.
     */
    public default double returnSkew()
    {
        return -1000.0;
    }

    /*
     * Returns ID of detected object.
     */
    public default int objID()
    {
        return -1000;
    }

    /*
     * Returns confidence of detected object.
     */
    public default float objConf()
    {
        return -1000.0f;
    }

    /*
     * Returns circumscribed area of detected object.
     */
    public default double objArea()
    {
        return -1000.0;
    }
}


