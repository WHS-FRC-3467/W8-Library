// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ComplexSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SimpleSubsystem;

public class RobotContainer {

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final Drivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  public final SimpleSubsystem simpleSubsystem = new SimpleSubsystem();
  public final ComplexSubsystem complexSubsystem = new ComplexSubsystem();



  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("Tests");

  private final Telemetry logger = new Telemetry(Constants.DriveConstants.MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand(drivetrain.run(
        () -> drivetrain.setControllerInput(-joystick.getLeftY(), -joystick.getLeftX(), -joystick.getRightX())));    

    drivetrain.registerTelemetry(logger::telemeterize);

    //joystick.povUp().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    joystick.a().whileTrue(complexSubsystem.setStateCommand(ComplexSubsystem.State.SCORE));
    joystick.b().whileTrue(simpleSubsystem.setStateCommand(SimpleSubsystem.State.ON));

  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return runAuto;
  }
}
