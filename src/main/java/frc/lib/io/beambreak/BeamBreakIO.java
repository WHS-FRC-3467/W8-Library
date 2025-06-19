// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.io.beambreak;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
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

  /** Gets the state of the beambreak. */
  public abstract boolean get();

  /** Applies a debounce to the beambreak state getter. */
  public boolean getDebounced();

  /**
   * @return getDebounced() if robot is real, return false if in sim
   */
  public boolean getDebouncedIfReal();

  /** Command that waits until the beambreak achieves a certain state */
  public Command stateWait(boolean state);

  /** Command that waits until the beambreak with the debouncer achieves a certain state */
  public Command stateWaitWithDebounce(boolean state);

  /**
   * Command that waits until the beambreak achieves a certain state if real If in sim, the command
   * waits for waitSecondsSim
   */
  public Command stateWaitIfReal(boolean state, double waitSecondsSim);

  /**
   * Command that waits until the beambreak with the debouncer achieves a certain state if real If
   * in sim, the command waits for waitSecondsSim
   */
  public Command stateWaitWithDebounceIfReal(boolean state, double waitSecondsSim);
}
