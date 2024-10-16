package frc.robot.subsystems.SampleRollers;

import frc.robot.subsystems.GenericRollerSubsystem.GenericRollerSubsystemIOTalonFX;

public class SampleRollersIOTalonFX extends GenericRollerSubsystemIOTalonFX implements SampleRollersIO {

  public SampleRollersIOTalonFX() {
    super(SampleRollersConstants.kIntakeConstants);
  }
}
