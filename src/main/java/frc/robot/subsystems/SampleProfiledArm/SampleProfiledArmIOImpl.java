package frc.robot.subsystems.SampleProfiledArm;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class SampleProfiledArmIOImpl extends GenericMotionProfiledSubsystemIOImpl implements SampleProfiledArmIO {

  public SampleProfiledArmIOImpl() {
    super(SampleProfiledArmConstants.kSubSysConstants, (Constants.currentMode == Mode.SIM));
  }
}
