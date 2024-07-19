// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class SimpleSubsystem extends SubsystemBase {

  @RequiredArgsConstructor
  @Getter
  public enum State {
    ON(() -> 1.0),
    OFF(() -> 0.0);

    private final DoubleSupplier outputSupplier;

    private double getStateOutput() {
      return outputSupplier.getAsDouble();
    }
  }

  @Getter
  @Setter
  private State state = State.OFF;

  /** Creates a new SimpleSubsystem. */
  public SimpleSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (state == State.OFF) {
      // m_intakeMotor.setControl(m_brake);
    } else {
      // m_intakeMotor.setControl(m_percent.withOutput(state.getStateOutput()));
    }
  }

  public Command setStateCommand(State state) {
    return runOnce(() -> this.state = state);
  }
}
