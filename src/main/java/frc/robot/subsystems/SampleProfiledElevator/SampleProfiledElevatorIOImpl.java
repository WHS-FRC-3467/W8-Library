package frc.robot.subsystems.SampleProfiledElevator;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class SampleProfiledElevatorIOImpl extends GenericMotionProfiledSubsystemIOImpl implements SampleProfiledElevatorIO {

  public SampleProfiledElevatorIOImpl() {
    super(SampleProfiledElevatorConstants.kSubSysConstants, (Constants.currentMode == Mode.SIM));
  }
}
