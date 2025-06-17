// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.io.beambreak;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface BeamBreakIO {

  @AutoLog
  abstract class BeamBreakIOInputs {
    /** Whether the BeamBreak is connected. */
    public boolean connected = false;
    /** Whether an object is within a specified range. */
    public boolean isDetected = false;
    /** The amount of ambient infrared light detected by the sensor. */
    public double ambientSignal = 0.0;
    /** The supply voltage (CANrange only). */
    public Voltage supplyVoltage = Volts.of(0.0);
    /** The signal strength of the sensor. */
    public double signalStrength = 0.0;
  }

  public void updateInputs(BeamBreakIOInputs inputs);

  public abstract boolean get();
}
