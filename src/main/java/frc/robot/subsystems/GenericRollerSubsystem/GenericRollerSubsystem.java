package frc.robot.subsystems.GenericRollerSubsystem;

import lombok.RequiredArgsConstructor;
import frc.robot.util.Alert;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@RequiredArgsConstructor
public abstract class GenericRollerSubsystem<G extends GenericRollerSubsystem.VoltageState> extends SubsystemBase {

  public interface VoltageState {
    double getOutput();
  }

  public abstract G getState();

  private final String name;
  private final GenericRollerSubsystemIO io;
  protected final GenericRollerIOInputsAutoLogged inputs = new GenericRollerIOInputsAutoLogged();
  private final Alert disconnected;

  private final boolean debug = true;
  
  public GenericRollerSubsystem(String name, GenericRollerSubsystemIO io) {
    this.name = name;
    this.io = io;

    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.WARNING);
  }

  public void periodic() {

    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconnected.set(!inputs.connected);

    io.runVolts(getState().getOutput());

    Logger.recordOutput("Rollers/" + name + "Goal", getState().toString());

    displayInfo(debug);
  }
  
  private void displayInfo(boolean debug) {
    if (debug) {
        SmartDashboard.putString(this.getClass().getSimpleName() + " State ", getState().toString());
        SmartDashboard.putNumber(this.getClass().getSimpleName() + " Setpoint ", getState().getOutput());
        SmartDashboard.putNumber(this.getClass().getSimpleName() + " Output ", inputs.appliedVoltage);
        SmartDashboard.putNumber(this.getClass().getSimpleName() + " Current Draw", inputs.supplyCurrentAmps);
    }
  }

}
