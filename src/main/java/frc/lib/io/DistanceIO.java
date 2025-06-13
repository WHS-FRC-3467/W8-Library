// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.io;

import static edu.wpi.first.units.Units.Millimeters;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public abstract class DistanceIO {
    private final String name;

    public DistanceIO(String name) {
        this.name = name;
    }

    @AutoLog
    abstract class DistanceSensorIOInputs {
        /** Whether the DistanceSensor is connected. */
        public boolean connected = false;
        /** Whether an object is within a specified range. */
        public boolean isDetected = false;
        /** Distance from the Distance sensor to the nearest object */
        public Distance distance = null;
        /** Standard deviation of the distance sensor measurement*/
        public Distance distanceStdDev = null;
        /** The amount of ambient infrared light detected by the DistanceSensor. */
        public double ambientSignal = 0.0;

    }

    public abstract Distance getDistance();

    public boolean isNearDistance(Distance expected, Distance tolerDistance) {
        return MathUtil.isNear(expected.in(Millimeters),getDistance().in(Millimeters),tolerDistance.in(Millimeters));
    }

}
