// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.io;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface DistanceIO {

    @AutoLog
    abstract class DistanceIOInputs {
        /** Whether the DistanceSensor is connected. */
        public boolean connected = false;
        /** Whether an object is within a specified range. */
        public boolean isDetected = false;
        /** Distance from the Distance sensor to the nearest object */
        public Distance distance = null;
        /** Standard deviation of the distance sensor measurement */
        public Distance distanceStdDev = null;
        /** The amount of ambient infrared light detected by the DistanceSensor. */
        public double ambientSignal = 0.0;
        /** The supply voltage of the DistanceSensor. */
        public Voltage supplyVoltage = Volts.of(0.0);
        /** The signal strength of the DistanceSensor */
        public double signalStrength = 0.0;
    }

    public void updateInputs(DistanceIOInputs inputs);

    public abstract boolean getBeamBreak();

    public abstract Distance getDistance();

    public boolean isNearDistance(Distance expected, Distance tolerDistance);
}
