package frc.robot.subsystems.SampleProfiledArm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;

@Setter
@Getter
public class SampleProfiledArm extends GenericMotionProfiledSubsystem<SampleProfiledArm.State> {

  @RequiredArgsConstructor
  @Getter
  public enum State implements TargetState {

      HOME(0.0, 0.0),
      LEVEL_1(Units.degreesToRotations(90.0), 0.0),
      LEVEL_2(Units.degreesToRotations(135.0), 0.0);

      private final double output;
      private final double feedFwd;
  }

  @Getter
  @Setter
  private State state = State.HOME;

  private final boolean debug = true;

  /** Constructor */
  public SampleProfiledArm(SampleProfiledArmIO io) {
    super(ProfileType.MM_POSITION, SampleProfiledArmConstants.kSubSysConstants, io);
  }

  public Command setStateCommand(State state) {
      displayInfo(debug);
      return startEnd(() -> this.state = state, () -> this.state = State.HOME);
  }

  private void displayInfo(boolean debug) {
    if (debug) {
      SmartDashboard.putString("SampleProfiledArm State ", getState().toString());
      SmartDashboard.putNumber("SampleProfiledArm Setpoint", Units.rotationsToDegrees(getState().getOutput()));
      SmartDashboard.putNumber("SampleProfiledArm Output", inputs.appliedVoltage[0]);
      SmartDashboard.putNumber("SampleProfiledArm Current Draw", inputs.supplyCurrentAmps[0]);
    }
  }
}
