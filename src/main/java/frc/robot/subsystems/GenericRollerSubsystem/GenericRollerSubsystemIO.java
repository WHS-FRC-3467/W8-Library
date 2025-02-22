package frc.robot.subsystems.GenericRollerSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface GenericRollerSubsystemIO {
  @AutoLog
  abstract class GenericRollerIOInputs {
    public boolean[] connected = new boolean[] {};
    public double[] positionRot = new double[] {};
    public double[] velocityRps = new double[] {};
    public double[] appliedVoltage = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] torqueCurrentAmps = new double[] {};
    public double[] tempCelsius = new double[] {};
  }

  default void updateInputs(GenericRollerIOInputs inputs) {}

  /** Run roller at specified voltage */
  default void runVolts(int rollerNum, double volts) {}

 	/** Stop in Open Loop */
	default public void stop() {}

}
