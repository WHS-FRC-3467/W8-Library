// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class IntakeSubsystem extends SubsystemBase {

  TalonFX m_intakeMotor = new TalonFX(0);
  private final DutyCycleOut m_percent = new DutyCycleOut(0);
  private final NeutralOut m_brake = new NeutralOut();

  @RequiredArgsConstructor
  @Getter
  public enum State {
    COLLECTING(() -> 0.7),
    EJECTING(() -> -0.3),
    OFF(() -> 0.0);

    private final DoubleSupplier outputSupplier;

    private double getStateOutput() {
      return outputSupplier.getAsDouble();
    }
  }

  @Getter
  @Setter
  private State state = State.OFF;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    var m_configuration = new TalonFXConfiguration();
    m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_configuration.Voltage.PeakForwardVoltage = 12.0;
    m_configuration.Voltage.PeakReverseVoltage = -12.0;
    m_intakeMotor.getConfigurator().apply(m_configuration);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    displayInfo(true);
    m_intakeMotor.setControl(m_percent.withOutput(state.getStateOutput()));
  }

  private void displayInfo(boolean debug) {
    if (debug) {
      SmartDashboard.putNumber("Intake Setpoint ", state.getStateOutput());
      SmartDashboard.putNumber("Intake Output ", m_intakeMotor.getMotorVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Intake Current Draw", m_intakeMotor.getSupplyCurrent().getValueAsDouble());
    }
    
  }
}
