// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class SimpleSubsystem extends SubsystemBase {

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

  private boolean debug = true;

  TalonFX m_motor = new TalonFX(Constants.ExampleSimpleSubsystemConstants.ID_Motor);
  private final DutyCycleOut m_percent = new DutyCycleOut(0);
  private final NeutralOut m_neutral = new NeutralOut();

  /** Creates a new SimpleSubsystem. */
  public SimpleSubsystem() {
    m_motor.getConfigurator().apply(Constants.ExampleSimpleSubsystemConstants.motorConfig());

  }

  @Override
  public void periodic() {

    if (state == State.OFF) {
      m_motor.setControl(m_neutral);
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
      SmartDashboard.putString(this.getClass().getSimpleName() + " State ", state.toString());
      SmartDashboard.putNumber(this.getClass().getSimpleName() + " Setpoint ", state.getOutput());
      SmartDashboard.putNumber(this.getClass().getSimpleName() + " Output ", m_motor.getMotorVoltage().getValueAsDouble());
      SmartDashboard.putNumber(this.getClass().getSimpleName() + " Current Draw", m_motor.getSupplyCurrent().getValueAsDouble());
    }

  }
}