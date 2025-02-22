package frc.robot.subsystems.SampleProfiledElevator;

import edu.wpi.first.wpilibj2.command.Command;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;

@Setter
@Getter
public class SampleProfiledElevator extends GenericMotionProfiledSubsystem<SampleProfiledElevator.State> {

  @RequiredArgsConstructor
  @Getter
  public enum State implements TargetState {

      HOME(0.0, 0.0),
      LEVEL_1(0.2, 0.0),
      LEVEL_2(0.6, 0.0);

      private final double output;
      private final double feedFwd;
  }

  @Getter
  @Setter
  private State state = State.HOME;

  /** Constructor */
  public SampleProfiledElevator(SampleProfiledElevatorIO io) {
    super(ProfileType.MM_POSITION, SampleProfiledElevatorConstants.kSubSysConstants, io);
  }

  public Command setStateCommand(State state) {
      return startEnd(() -> this.state = state, () -> this.state = State.HOME);
  }

}
