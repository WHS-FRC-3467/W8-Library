// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Shooter extends SubsystemBase {

  @RequiredArgsConstructor
  @Getter
  public enum State {
    OFF(0.0, 0.0),
    SHOOT(70.0, 40.0);

    private final double leftVelocity;
    private final double rightVelocity;
  }

  @Getter
  @Setter
  private State state = State.OFF;

  private double tolerance = 2; //TODO: Confirm RPM

  TalonFX m_leftShooterMotor = new TalonFX(ShooterConstants.ID_ShooterLeft);
  TalonFX m_rightShooterMotor = new TalonFX(ShooterConstants.ID_ShooterRight);
  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0.0);
  private final NeutralOut m_neutralOut = new NeutralOut();

  /** Creates a new SimpleSubsystem. */
  public Shooter() {

    m_leftShooterMotor.getConfigurator().apply(ShooterConstants.motorConfig());
    m_rightShooterMotor.getConfigurator().apply(ShooterConstants.motor2Config());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (state == State.OFF) {
      m_leftShooterMotor.setControl(m_neutralOut);
      m_rightShooterMotor.setControl(m_neutralOut);
    } else {
      m_leftShooterMotor.setControl(m_voltageVelocity.withVelocity(state.getLeftVelocity()));
      m_rightShooterMotor.setControl(m_voltageVelocity.withVelocity(state.getRightVelocity()));
    }

    displayInfo(true);
  }

  public boolean atGoal() {
    return state == State.OFF || (
          MathUtil.isNear(state.getLeftVelocity(), m_leftShooterMotor.getVelocity().getValueAsDouble(), tolerance) &&
          MathUtil.isNear(state.getRightVelocity(), m_rightShooterMotor.getVelocity().getValueAsDouble(), tolerance));
  }

  public Command setStateCommand(State state) {
    return startEnd(() -> setState(state),() -> setState(State.OFF));
  }

  private void displayInfo(boolean debug) {
    if (debug) {
      SmartDashboard.putString("Shooter State ", state.toString());
      SmartDashboard.putNumber("Shooter Left Setpoint ", state.getLeftVelocity());
      SmartDashboard.putNumber("Shooter Right Setpoint ", state.getRightVelocity());
      SmartDashboard.putNumber("Shooter Left Speed", m_leftShooterMotor.getVelocity().getValueAsDouble());
      SmartDashboard.putNumber("Shooter Right Speed", m_rightShooterMotor.getVelocity().getValueAsDouble());
      SmartDashboard.putBoolean("Shooter at Goal?", atGoal());
      SmartDashboard.putNumber("Intake Current Draw", m_leftShooterMotor.getSupplyCurrent().getValueAsDouble());
    }

  }
}
