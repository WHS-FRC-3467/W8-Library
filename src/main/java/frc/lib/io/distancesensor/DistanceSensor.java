// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.io.distancesensor;

import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

/** Standardized interface for distance sensors used in FRC. */
public interface DistanceSensor {

    @AutoLog
    abstract class DistanceSensorInputs {
        /** Whether the sensor is connected. */
        public boolean connected = false;
        /** Distance from the sensor to the nearest object */
        public Distance distance = null;
        /** The amount of ambient infrared light detected by the sensor. */
        public double ambientSignal = 0.0;
    }

    /**
     * Getter for the name of the sensor
     * 
     * @return The name of the sensor
     */
    public default String getName()
    {
        return "";
    }

    /**
     * Updates the provided {@link DistanceSensorInputs} instance with the latest sensor readings.
     * If the sensor is not connected, it populates the fields with default values.
     *
     * @param inputs The structure to populate with updated sensor values.
     */
    public default void updateInputs(DistanceSensorInputs inputs)
    {}
}
