package frc.robot.subsystems.GenericRollerSubsystem;

import frc.robot.util.Alert;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class GenericRollerSubsystem<G extends GenericRollerSubsystem.VoltageState> extends SubsystemBase {

  public interface VoltageState {

    public double getOutput(int index);
  }

  public abstract G getState();

  private final String name;
  private final int numRollers;
  private final GenericRollerSubsystemIO io;
  protected final GenericRollerIOInputsAutoLogged inputs = new GenericRollerIOInputsAutoLogged();
  private final List<Alert> disconnected = new ArrayList<Alert> ();

  private final boolean debug = true;
  
  public GenericRollerSubsystem(String name, int numRollers, GenericRollerSubsystemIO io) {
    this.name = name;
    this.numRollers = numRollers;
    this.io = io;

    // Set up a disconnection Alert for each roller motor
    for (int i = 0; i < numRollers; i++) {
      this.disconnected.add(new Alert(name + " motor " + i + " disconnected!", Alert.AlertType.WARNING));
    }
  }

  public void periodic() {

    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    // Check and run each roller motor
    for (int i = 0; i < numRollers; i++) {
      // Check for motor disconnection
      disconnected.get(i).set(!inputs.connected[i]);
      // Run motor by voltage
      io.runVolts(i, getState().getOutput(i));
    }

    Logger.recordOutput("Rollers/" + name + "Goal", getState().toString());

    displayInfo(debug);
  }
  
  private void displayInfo(boolean debug) {
    if (debug) {
      SmartDashboard.putString(this.name + " State ", getState().toString());
        
      for (int i = 0; i < 0; i++) {
        SmartDashboard.putNumber(this.name + " Setpoint " + i, getState().getOutput(i));
        SmartDashboard.putNumber(this.name + " Output " + i, inputs.appliedVoltage[i]);
        SmartDashboard.putNumber(this.name + " Current Draw" + i, inputs.supplyCurrentAmps[i]);
      }
    }
  }

}
