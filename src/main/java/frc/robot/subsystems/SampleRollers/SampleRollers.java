package frc.robot.subsystems.SampleRollers;

import edu.wpi.first.wpilibj2.command.Command;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

import frc.robot.subsystems.GenericRollerSubsystem.GenericRollerSubsystem;
import frc.robot.subsystems.GenericRollerSubsystem.GenericRollerSubsystem.VoltageState;

@Setter
@Getter
public class SampleRollers extends GenericRollerSubsystem<SampleRollers.State> {

  @RequiredArgsConstructor
  @Getter
  public enum State implements VoltageState {
      OFF(0.0),
      INTAKE(10.0),
      EJECT(-8.0);

      private final double output;
  }

  @Getter
  @Setter
  private State state = State.OFF;

  /** Constructor */
  public SampleRollers(SampleRollersIO io) {
    super("SampleRollers", io);
  }

  public Command setStateCommand(State state) {
      return startEnd(() -> this.state = state, () -> this.state = State.OFF);
  }

}
