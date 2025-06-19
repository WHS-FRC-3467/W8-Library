package frc.lib.io.beambreak;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.util.LoggedTunableBoolean;

public class BeamBreakIOSim implements BeamBreakIO {

  private final Debouncer debouncer;
  private final String name;
  private final LoggedTunableBoolean button;

  /* Class to represent beambreaks or limit switches in simulation */
  public BeamBreakIOSim(Time debounce, String name) {

    debouncer = new Debouncer(debounce.in(Units.Seconds), DebounceType.kBoth);
    this.name = name;
    this.button = new LoggedTunableBoolean(name + "/BeamBreak", false);
  }

  /**
   * Updates the BeamBreakIO inputs
   *
   * @param inputs The BeamBreakIOInputs object where the updated values will be stored.
   */
  @Override
  public void updateInputs(BeamBreakIOInputs inputs) {

    inputs.isDetected = button.getAsBoolean();
  }

  @Override
  public boolean get() {
    return button.getAsBoolean();
  }

  @Override
  public boolean getDebounced() {
    return debouncer.calculate(get());
  }

  @Override
  public boolean getDebouncedIfReal() {
    return Robot.isReal() && getDebounced();
  }

  @Override
  public Command stateWait(boolean state) {
    return Commands.waitUntil(() -> get() == state);
  }

  @Override
  public Command stateWaitWithDebounce(boolean state) {
    return Commands.waitUntil(() -> getDebounced() == state);
  }

  @Override
  public Command stateWaitIfReal(boolean state, double waitSecondsSim) {
    return Commands.either(
        stateWait(state), Commands.waitSeconds(waitSecondsSim), () -> Robot.isReal());
  }

  @Override
  public Command stateWaitWithDebounceIfReal(boolean state, double waitSecondsSim) {
    return Commands.either(
        stateWaitWithDebounce(state), Commands.waitSeconds(waitSecondsSim), () -> Robot.isReal());
  }
}
