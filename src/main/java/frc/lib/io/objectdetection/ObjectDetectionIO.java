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

package frc.lib.io.objectdetection;

import java.util.List;
import org.littletonrobotics.junction.AutoLog;

/**
 * Standardized interface for ObjectDetection-IO used in FRC. This interface is often implemented
 * through an ML pipeline.
 */
public interface ObjectDetectionIO {

    /*
     * Abstract class defining data type for updateInputs method.
     */
    @AutoLog
    abstract class ObjectDetectionIOInputs {
        /** Whether the camera is connected. */
        public boolean connected = false;
        /**
         * Each index of latestTargetObservations is a single TargetObservation (defined below) with
         * members for objID, objConf, etc., effectively acting as a 2D array.
         */
        public TargetObservation[] latestTargetObservations = new TargetObservation[0];
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
        double skew,
        /** X-coord & Y-coord of bounding box corner 1. */
        List<Double> cornerOne,
        /** X-coord & Y-coord of bounding box corner 2. */
        List<Double> cornerTwo,
        /** X-coord & Y-coord of bounding box corner 3. */
        List<Double> cornerThree,
        /** X-coord & Y-coord of bounding box corner 4. */
        List<Double> cornerFour) {
    }

    /*
     * Name of the camera capturing optical data.
     */
    public default String getCamera() {
        return "";
    }

    /*
     * Updates the provided ObjectDetectionIOInputs object using the latest camera readings. If the
     * camera is not connected, the ObjectDetectionIOInput fields remain empty.
     */
    public default void updateInputs(ObjectDetectionIOInputs inputs) {}
}
