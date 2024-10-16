package frc.robot.subsystems.SampleRollers;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.GenericRollerSubsystem.GenericRollerSubsystemIOSim;

public class SampleRollersIOSim extends GenericRollerSubsystemIOSim implements SampleRollersIO {
  private static final DCMotor motorModel = DCMotor.getKrakenX60Foc(1);
  private static final double reduction = (18.0 / 12.0);
  private static final double moi = 0.001;

  public SampleRollersIOSim() {
    super(motorModel, reduction, moi);
  }
}
