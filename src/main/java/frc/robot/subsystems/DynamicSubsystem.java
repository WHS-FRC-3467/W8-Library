// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExampleComplexSubsystemConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class DynamicSubsystem extends SubsystemBase {

  @RequiredArgsConstructor
  @Getter
  public enum State {
    HOME(() -> 0.0),
    SCORE(() -> 90.0), //Static angle
    AIM(() -> 10.0); //Dynamic aiming

    private final DoubleSupplier outputSupplier;

    private double getStateOutput() {
      return Units.degreesToRadians(outputSupplier.getAsDouble());
    }
  }

  @Getter
  @Setter
  private State state = State.HOME;

  TalonFX m_motor = new TalonFX(ExampleComplexSubsystemConstants.ID_Motor);
  private final static MotionMagicVoltage m_magic = new MotionMagicVoltage(0);
  private final static PositionVoltage m_position = new PositionVoltage(0);
  private final NeutralOut m_neutral = new NeutralOut();

  private double goalAngle;


  /** Creates a new ComplexSubsystem. */
  public DynamicSubsystem() {
    m_motor.getConfigurator().apply(ExampleComplexSubsystemConstants.motorConfig());
  }

  @Override
  public void periodic() {
    goalAngle = MathUtil.clamp(state.getStateOutput(), ExampleComplexSubsystemConstants.lowerLimit, ExampleComplexSubsystemConstants.upperLimit);

    if (state == State.HOME && atGoal()) {
      m_motor.setControl(m_neutral);
    } else if (state == State.AIM) {
      m_motor.setControl(m_position.withPosition(goalAngle));
    } else {
      m_motor.setControl(m_magic.withPosition(goalAngle));
    }

    displayInfo(true);
  }

  public boolean atGoal() {
    return Math.abs(state.getStateOutput() - m_motor.getPosition().getValueAsDouble()) < ExampleComplexSubsystemConstants.tolerance;
  }

  public Command setStateCommand(State state) {
    return startEnd(() -> this.state = state, () -> this.state = State.HOME);
  }

  private void displayInfo(boolean debug) {
    if (debug) {
      SmartDashboard.putString(this.getClass().getSimpleName() + " State ", state.toString());
      SmartDashboard.putNumber(this.getClass().getSimpleName() + " Setpoint ", state.getStateOutput());
      SmartDashboard.putNumber(this.getClass().getSimpleName() + " Output ", m_motor.getMotorVoltage().getValueAsDouble());
      SmartDashboard.putNumber(this.getClass().getSimpleName() + " Current Draw", m_motor.getSupplyCurrent().getValueAsDouble());
      SmartDashboard.putBoolean(this.getClass().getSimpleName() + " atGoal", atGoal());
    }

  }
}
