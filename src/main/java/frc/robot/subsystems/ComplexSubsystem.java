// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class ComplexSubsystem extends SubsystemBase {

  @RequiredArgsConstructor
  @Getter
  public enum State {
    HOME(() -> 0.0),
    OUT(() -> 90.0);

    private final DoubleSupplier outputSupplier;

    private double getStateOutput() {
      return outputSupplier.getAsDouble();
    }
  }

  @Getter
  @Setter
  private State state = State.HOME;

  private final double upperLimitDegrees = 180;
  private final double lowerLimitDegrees = 0;
  private final double maxVelocity = 1;
  private final double maxAcceleration = 1;
  private ProfiledPIDController pidController = new ProfiledPIDController(0, 0, 0,
      new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
  private double goalAngle;
  private double currentAngle;
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0);
  private double output = 0;

  //TalonFX m_motor = new TalonFX(Constants.ExampleSubsystemConstants.ID_Motor);
  private final DutyCycleOut m_voltage = new DutyCycleOut(0);
  private final NeutralOut m_brake = new NeutralOut();

  /** Creates a new ComplexSubsystem. */
  public ComplexSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    goalAngle = MathUtil.clamp(state.getStateOutput(), lowerLimitDegrees, upperLimitDegrees);

    if (state == State.HOME && pidController.atGoal()) {
      // motor.setControl(Neutral)
    } else {
      output = pidController.calculate(currentAngle, goalAngle) + ff.calculate(0, 0);
      // motor.setControl(output);
    }

  }

  public boolean atGoal() {
    return pidController.atGoal();
  }

  public Command setStateCommand(State state) {
    return startEnd(() -> this.state = state, () -> this.state = State.HOME);
  }

    private void displayInfo(boolean debug) {
    if (debug) {
      SmartDashboard.putString("ComplexSubsystem State ", state.toString());
      SmartDashboard.putNumber("ComplexSubsystem Setpoint ", Units.radiansToDegrees(goalAngle));
      //SmartDashboard.putNumber("ComplexSubsystem Angle ", Units.radiansToDegrees(m_encoder.getDistance()));
      SmartDashboard.putBoolean("ComplexSubsystem at Goal?", atGoal());
      //SmartDashboard.putNumber("ComplexSubsystem Current Draw", m_armMotor.getSupplyCurrent().getValueAsDouble());
      SmartDashboard.putData("ComplexSubsystem PID",pidController);
    }

  }
}
