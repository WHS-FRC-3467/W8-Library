// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class ArmSubsystem extends SubsystemBase {

  @RequiredArgsConstructor
  @Getter
  public enum State {
    STOW(() -> 0.0),
    INTAKE(() -> 1.0),
    SUBWOOFER(() -> 1.0),
    AMP(() -> 93.0),
    FEED(() -> 10.0),
    CLIMB(() -> 88.0),
    HARMONY(() -> 122.0),
    LOOKUP(() -> 93.0);

    private final DoubleSupplier outputSupplier;

    private double getStateOutput() {
      return Units.degreesToRadians(outputSupplier.getAsDouble());
    }
  }

  @Getter
  @Setter
  private State state = State.STOW;

  private final double upperLimit = Units.degreesToRadians(130);
  private final double lowerLimit = Units.degreesToRadians(0.0);
  private final double tolerance = Units.degreesToRadians(.5);
  private final double maxVelocity = Math.PI;
  private final double maxAcceleration = Math.PI / 2;

  private ProfiledPIDController pidController = new ProfiledPIDController(0, 0, 0,
      new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
  private double goalAngle;
  private double currentAngle;
  private ArmFeedforward ff = new ArmFeedforward(0, 0, 0);
  private double output = 0;

  TalonFX m_armMotor = new TalonFX(0);
  private VoltageOut m_VoltageOutput = new VoltageOut(0.0);
  private final NeutralOut m_brake = new NeutralOut();

  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(0);
  


  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    pidController.setTolerance(tolerance);
    pidController.setIZone(100); // TODO: Figure out actual value

    var m_configuration = new TalonFXConfiguration();
    m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_configuration.Voltage.PeakForwardVoltage = 12.0;
    m_configuration.Voltage.PeakReverseVoltage = -12.0;
    m_armMotor.getConfigurator().apply(m_configuration);

    m_encoder.setDutyCycleRange(1.0/1025.0, 1024.0/1025.0);
    m_encoder.setDistancePerRotation(2*Math.PI);
    m_encoder.setPositionOffset(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    goalAngle = MathUtil.clamp(state.getStateOutput(), lowerLimit, upperLimit);

    if (state == State.STOW && pidController.atGoal()) {
      m_armMotor.setControl(m_brake);
    } else {
      output = pidController.calculate(m_encoder.getDistance(), goalAngle) + ff.calculate(goalAngle, 0);
      m_armMotor.setControl(m_VoltageOutput.withOutput(output));
    }

  }

  public Command setStateCommand(State state) {
    return runOnce(() -> this.state = state);
  }
}
