package frc.robot.subsystems.GenericRollerSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface GenericRollerSubsystemIO {
  @AutoLog
  abstract class GenericRollerIOInputs {
    public boolean connected = true;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(GenericRollerIOInputs inputs) {}

  /** Run roller at specified voltage */
  default void runVolts(double volts) {}

 	/** Stop in Open Loop */
	default public void stop() {}

}
