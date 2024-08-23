// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.ArmConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Arm extends SubsystemBase {

  @RequiredArgsConstructor
  @Getter
  public enum State { //0 angle is straight horizontal, -20 is on the hardstops
    STOW(() -> 0.0),
    SCORE(() -> 90.0);
    //LOOKUP(() -> RobotState.getInstance().getShotAngle());

    private final DoubleSupplier outputSupplier;

    private double getStateOutput() {
      return Units.degreesToRadians(outputSupplier.getAsDouble());
    }
  }

  @Getter
  @Setter
  private State state = State.STOW;


  //Create a on RIO Profile PID Controller, adding constraints to limit max vel and accel 
  private ProfiledPIDController pidController = new ProfiledPIDController(18, 0, 0.2,
      new TrapezoidProfile.Constraints(ArmConstants.maxVelocity, ArmConstants.maxAcceleration));

  private ArmFeedforward ff = new ArmFeedforward(0.5, 0.4, 2.5, 0.01);

  private double goalAngle;
  private double output = 0;

  TalonFX m_armMotor = new TalonFX(ArmConstants.ID_ArmLeader);
  TalonFX m_armFollowerMotor = new TalonFX(ArmConstants.ID_ArmFollower);

  private VoltageOut m_VoltageOutput = new VoltageOut(0.0);
  private NeutralOut m_neutralOut = new NeutralOut();

  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(ArmConstants.ID_ArmAbsEncoder);

  private Debouncer m_debounce = new Debouncer(.1);
  


  /** Creates a new ArmSubsystem. */
  public Arm() {

    m_armMotor.getConfigurator().apply(ArmConstants.motorConfig());
    m_armFollowerMotor.getConfigurator().apply(ArmConstants.motorConfig());
    m_armFollowerMotor.setControl(new Follower(ArmConstants.ID_ArmLeader, true));

    m_encoder.setDutyCycleRange(1.0/1025.0, 1024.0/1025.0); 
    m_encoder.setDistancePerRotation(2*Math.PI); //Define 1 full rotation to be 2 Pi
    m_encoder.setPositionOffset(0.50666666); //Encoder offset to make horizontal report 0 deg
    

    pidController.setTolerance(ArmConstants.tolerance);
    pidController.setIZone(100); // TODO: Figure out actual value

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    goalAngle = MathUtil.clamp(state.getStateOutput(), ArmConstants.lowerLimit, ArmConstants.upperLimit);
    pidController.setGoal(goalAngle);
    
    if (state == State.STOW && atGoal()) {
      m_armMotor.setControl(m_neutralOut);
    } else {
      output = pidController.calculate(m_encoder.getDistance()) + ff.calculate(goalAngle, 0);
      m_armMotor.setControl(m_VoltageOutput.withOutput(output));
    }

    displayInfo(true);

  }

  
  public boolean atGoal() {
    //return m_debounce.calculate(MathUtil.isNear(goalAngle, m_encoder.getDistance(), ArmConstants.tolerance));
    return m_debounce.calculate(pidController.atGoal());
    //return pidController.atGoal();
  }

  public Command setStateCommand(State state) {
    return startEnd(() -> setState(state), () -> setState(State.STOW))
        .withName("Set Arm State: " + state.toString());
  }

  private void displayInfo(boolean debug) {
    if (debug) {
      SmartDashboard.putString("Arm State ", state.toString());
      SmartDashboard.putNumber("Arm Setpoint ", Units.radiansToDegrees(goalAngle));
      SmartDashboard.putNumber("Arm Angle ", Units.radiansToDegrees(m_encoder.getDistance()));
      SmartDashboard.putBoolean("Arm at Goal?", atGoal());
      SmartDashboard.putNumber("Arm Current Draw", m_armMotor.getSupplyCurrent().getValueAsDouble());
      SmartDashboard.putData("Arm PID",pidController);
    }

  }
}
