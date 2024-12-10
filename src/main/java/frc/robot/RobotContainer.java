// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ComplexSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SimpleSubsystem;
import frc.robot.subsystems.SampleProfiledArm.SampleProfiledArmIO;
import frc.robot.subsystems.SampleProfiledArm.SampleProfiledArmIOImpl;
import frc.robot.subsystems.SampleProfiledElevator.SampleProfiledElevator;
import frc.robot.subsystems.SampleProfiledElevator.SampleProfiledElevatorIO;
import frc.robot.subsystems.SampleProfiledElevator.SampleProfiledElevatorIOImpl;
import frc.robot.subsystems.SampleProfiledArm.SampleProfiledArm;
import frc.robot.subsystems.SampleRollers.SampleRollers;
import frc.robot.subsystems.SampleRollers.SampleRollersIO;
import frc.robot.subsystems.SampleRollers.SampleRollersIOImpl;

public class RobotContainer {

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final Drivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  // Subsystems
  //public final SimpleSubsystem simpleSubsystem = new SimpleSubsystem();
  //public final ComplexSubsystem complexSubsystem = new ComplexSubsystem();
  //public final SampleRollers sampleRollersSubsystem;
  public final SampleProfiledArm sampleArmSubsystem;
  //public final SampleProfiledElevator sampleElevatorSubsystem;

  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("Tests");

  private final Telemetry logger = new Telemetry(Constants.DriveConstants.MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand(drivetrain.run(
        () -> drivetrain.setControllerInput(-joystick.getLeftY(), -joystick.getLeftX(), -joystick.getRightX())));    

    drivetrain.registerTelemetry(logger::telemeterize);

    //joystick.povUp().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // Buttons A & B run the Complex and Simple subsystems when held
    //joystick.a().whileTrue(complexSubsystem.setStateCommand(ComplexSubsystem.State.SCORE));
    //joystick.b().whileTrue(simpleSubsystem.setStateCommand(SimpleSubsystem.State.ON));

    // Buttons X & Y run the Sample Rollers when held
    //joystick.x().whileTrue(sampleRollersSubsystem.setStateCommand(SampleRollers.State.EJECT));
    //joystick.y().whileTrue(sampleRollersSubsystem.setStateCommand(SampleRollers.State.INTAKE));
    
    // POV Down brings Arm and Elevator to Home position
    joystick.povDown().onTrue(
      Commands.parallel(
        sampleArmSubsystem.setStateCommand(SampleProfiledArm.State.HOME)//,
        //sampleElevatorSubsystem.setStateCommand(SampleProfiledElevator.State.HOME)
      )
    );

    // POV Left sends Arm and Elevator to LEVEL_1
    joystick.povLeft().onTrue(
      Commands.parallel(
        sampleArmSubsystem.setStateCommand(SampleProfiledArm.State.LEVEL_1)//,
        //sampleElevatorSubsystem.setStateCommand(SampleProfiledElevator.State.LEVEL_1)
      )
    );

    // POV Up sends Arm and Elevator to LEVEL_2
    joystick.povUp().onTrue(
      Commands.parallel(
        sampleArmSubsystem.setStateCommand(SampleProfiledArm.State.LEVEL_2)//,
        //sampleElevatorSubsystem.setStateCommand(SampleProfiledElevator.State.LEVEL_2)
      )
    );

  }

  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
      case SIM:
        // Real or Simulated robot
        // Hardware or physics sim IO implementation used will depend on Constants.currentMode
        //sampleRollersSubsystem = new SampleRollers(new SampleRollersIOImpl());
        sampleArmSubsystem = new SampleProfiledArm(new SampleProfiledArmIOImpl());
        //sampleElevatorSubsystem = new SampleProfiledElevator(new SampleProfiledElevatorIOImpl());
        break;

      default:
        // Replayed robot, disable IO implementations
        //sampleRollersSubsystem = new SampleRollers(new SampleRollersIO() {});
        sampleArmSubsystem = new SampleProfiledArm(new SampleProfiledArmIO() {});
        //sampleElevatorSubsystem = new SampleProfiledElevator(new SampleProfiledElevatorIO() {});
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
