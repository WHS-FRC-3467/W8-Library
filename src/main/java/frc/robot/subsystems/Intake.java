// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;


public class Intake extends SubsystemBase {

  @RequiredArgsConstructor
  @Getter
  public enum State {
    INTAKE(0.5),
    EJECTING(-0.3),
    OFF(0.0);

    private final double outputSupplier;

  }

  @Getter
  @Setter
  private State state = State.OFF;

  TalonFX m_intakeMotor = new TalonFX(IntakeConstants.ID_IntakeMotor);
  TalonFX m_intakeFollowerMotor = new TalonFX(IntakeConstants.ID_IntakeFollower);
  private final DutyCycleOut m_percent = new DutyCycleOut(0);
  private final NeutralOut m_neutralOutput = new NeutralOut();

  /** Creates a new IntakeSubsystem. */
  public Intake() {
    
    m_intakeMotor.getConfigurator().apply(IntakeConstants.motorConfig());
    m_intakeFollowerMotor.getConfigurator().apply(IntakeConstants.motorConfig());
    m_intakeFollowerMotor.setControl(new Follower(IntakeConstants.ID_IntakeMotor, false));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (state == State.OFF) {
      m_intakeMotor.setControl(m_neutralOutput);
    } else {
      m_intakeMotor.setControl(m_percent.withOutput(state.getOutputSupplier()));
    }

    displayInfo(true);

  }

  public boolean atGoal() {
    return (m_intakeMotor.getDutyCycle().getValueAsDouble() == state.getOutputSupplier());
  }

  private void displayInfo(boolean debug) {
    if (debug) {
      SmartDashboard.putString("Intake State ", state.toString());
      SmartDashboard.putNumber("Intake Setpoint ", state.getOutputSupplier());
      SmartDashboard.putNumber("Intake Output ", m_intakeMotor.getMotorVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Intake Current Draw", m_intakeMotor.getSupplyCurrent().getValueAsDouble());
    }

  }

  public Command setStateCommand(State state) {
    return startEnd(() -> setState(state),() -> setState(State.OFF));
  }
}
