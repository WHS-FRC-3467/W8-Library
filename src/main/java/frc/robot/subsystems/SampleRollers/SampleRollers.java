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

  // This is the number of independent roller motors in the subsystem
  // It must equal the size of the voltage arrays in the State enum
  // It also must equal the length of the kMotorIds List in the supplied Constants file
  public static int numRollers = 2;

  @RequiredArgsConstructor
  @Getter
  public enum State implements VoltageState {

      OFF(new double[] {0.0, 0.0}),
      INTAKE(new double[] {10.0, -5.0}),
      EJECT(new double[] {-8.0, 4.0});

      private final double[] output;

      public double getOutput(int index) {
        return this.output[index];
      }
   
  }

  @Getter
  @Setter
  private State state = State.OFF;

  /** Constructor */
  public SampleRollers(SampleRollersIO io) {
    super("SampleRollers", numRollers, io);
  }

  public Command setStateCommand(State state) {
      return startEnd(() -> this.state = state, () -> this.state = State.OFF);
  }

}
