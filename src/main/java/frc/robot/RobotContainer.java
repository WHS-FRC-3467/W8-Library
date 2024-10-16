// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ComplexSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SimpleSubsystem;
import frc.robot.subsystems.SampleRollers.SampleRollers;
import frc.robot.subsystems.SampleRollers.SampleRollersIO;
import frc.robot.subsystems.SampleRollers.SampleRollersIOSim;
import frc.robot.subsystems.SampleRollers.SampleRollersIOTalonFX;

public class RobotContainer {

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final Drivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  // Subsystems
  public final SimpleSubsystem simpleSubsystem = new SimpleSubsystem();
  public final ComplexSubsystem complexSubsystem = new ComplexSubsystem();
  public final SampleRollers sampleRollersSubsystem;

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

    joystick.x().whileTrue(sampleRollersSubsystem.setStateCommand(SampleRollers.State.EJECT));

  }

  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        sampleRollersSubsystem = new SampleRollers(new SampleRollersIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        sampleRollersSubsystem = new SampleRollers(new SampleRollersIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        sampleRollersSubsystem = new SampleRollers(new SampleRollersIO() {});
        break;
    }

    configureBindings();

    // Detect if controllers are missing / Stop multiple warnings
    DriverStation.silenceJoystickConnectionWarning(true);

  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return runAuto;
  }
}
