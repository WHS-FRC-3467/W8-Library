package frc.robot.subsystems.SampleRollers;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.GenericRollerSubsystem.GenericRollerSubsystemIOImpl;

public class SampleRollersIOImpl extends GenericRollerSubsystemIOImpl implements SampleRollersIO {

  public SampleRollersIOImpl() {
    super(SampleRollersConstants.kSubSysConstants, (Constants.currentMode == Mode.SIM));
  }
}
