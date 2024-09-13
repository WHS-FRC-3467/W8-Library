// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SimpleSubsystem;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class SimpleSubsystem extends SubsystemBase {

  private final SimpleSubsystemIO io;
  private final SimpleSubsystemIOInputsAutoLogged inputs = new SimpleSubsystemIOInputsAutoLogged();
  
  @RequiredArgsConstructor
  @Getter
  public enum State {
    ON(1.0),
    OFF(0.0);

    private final double output;
  }

  @Getter
  @Setter
  private State state = State.OFF;

  private boolean debug = false;

  TalonFX m_motor = new TalonFX(Constants.ExampleSubsystemConstants.ID_Motor);
  private final DutyCycleOut m_percent = new DutyCycleOut(0);
  private final NeutralOut m_brake = new NeutralOut();

  /** Creates a new SimpleSubsystem. */
  public SimpleSubsystem(SimpleSubsystemIO io) {
    this.io = io;

    // We need to establish what mode the software is running on to determine if PID needs to change
    // There are different PID values and FeedForward values since the physics simulator does not fully match
    // the real robot
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        io.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        break;
    }

    // MJW 9/13/2024: We no longer establish the hardware settings in the constants folder
    // instead the user will define the motor settings inside of the IO layer TalonFx
    //m_motor.getConfigurator().apply(Constants.ExampleSubsystemConstants.motorConfig());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);  // Goes to the IO layer and updates the variables from Hardware/Sim
    Logger.processInputs("SimpleSubsystem", inputs);  //Updates the inputs in the logger

    if (state == State.OFF) {
      m_motor.setControl(m_brake);
    } else {
      m_motor.setControl(m_percent.withOutput(state.getOutput()));
    }

    displayInfo(debug);
  }

  public Command setStateCommand(State state) {
    return startEnd(() -> this.state = state, () -> this.state = State.OFF);
  }

  private void displayInfo(boolean debug) {
    if (debug) {
      SmartDashboard.putString("SimpleSubsystem State ", state.toString());
      SmartDashboard.putNumber("SimpleSubsystem Setpoint ", state.getOutput());
      SmartDashboard.putNumber("SimpleSubsystem Output ", m_motor.getMotorVoltage().getValueAsDouble());
      SmartDashboard.putNumber("SimpleSubsystem Current Draw", m_motor.getSupplyCurrent().getValueAsDouble());
    }

  }
}
