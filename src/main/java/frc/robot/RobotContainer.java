/*
 * Copyright (C) 2025 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.util.CommandXboxControllerExtended;
import frc.robot.Constants.PathConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.OnTheFlyPathCommand;
import frc.robot.subsystems.beambreak1.BeamBreak1;
import frc.robot.subsystems.beambreak1.BeamBreak1Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelConstants;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDsConstants;
import frc.robot.subsystems.rotary.RotarySubsystem;
import frc.robot.subsystems.rotary.RotarySubsystemConstants;
import frc.robot.subsystems.servo1.Servo1;
import frc.robot.subsystems.servo1.Servo1Constants;
import frc.robot.subsystems.lasercan1.LaserCAN1;
import frc.robot.subsystems.lasercan1.LaserCAN1Constants;
import java.util.ArrayList;
import java.util.Arrays;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
    // Subsystems
    public final Drive drive;
    private final LEDs leds;
    private final LaserCAN1 laserCAN1;
    private final BeamBreak1 beamBreak1;
    private final Servo1 servo1;
    private final Flywheel flywheel;
    private final RotarySubsystem rotary;

    // Controller
    private final CommandXboxControllerExtended controller = new CommandXboxControllerExtended(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        switch (Constants.currentMode) {
            case REAL -> {
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOTalonFX(DriveConstants.FrontLeft),
                    new ModuleIOTalonFX(DriveConstants.FrontRight),
                    new ModuleIOTalonFX(DriveConstants.BackLeft),
                    new ModuleIOTalonFX(DriveConstants.BackRight));

                leds = new LEDs(LEDsConstants.getLightsIOReal());
                laserCAN1 = new LaserCAN1(LaserCAN1Constants.getReal());
                beamBreak1 = new BeamBreak1(BeamBreak1Constants.getReal());
                servo1 = new Servo1(Servo1Constants.getReal());
                flywheel = new Flywheel(FlywheelConstants.getReal());
                rotary = new RotarySubsystem(RotarySubsystemConstants.getReal());
            }

            case SIM -> {
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(DriveConstants.FrontLeft),
                    new ModuleIOSim(DriveConstants.FrontRight),
                    new ModuleIOSim(DriveConstants.BackLeft),
                    new ModuleIOSim(DriveConstants.BackRight));

                leds = new LEDs(LEDsConstants.getLightsIOSim());
                laserCAN1 =
                    new LaserCAN1(LaserCAN1Constants.getSim());
                beamBreak1 = new BeamBreak1(
                    BeamBreak1Constants.getSim());
                servo1 = new Servo1(Servo1Constants.getSim());
                flywheel = new Flywheel(FlywheelConstants.getSim());
                rotary = new RotarySubsystem(RotarySubsystemConstants.getSim());
            }

            default -> {
                // Replayed robot, disable IO implementations
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {});

                leds = new LEDs(LEDsConstants.getLightsIOReplay());
                laserCAN1 =
                    new LaserCAN1(LaserCAN1Constants.getReplay());
                beamBreak1 =
                    new BeamBreak1(BeamBreak1Constants.getReplay());
                servo1 = new Servo1(Servo1Constants.getReplay());
                flywheel = new Flywheel(FlywheelConstants.getReplay());
                rotary = new RotarySubsystem(RotarySubsystemConstants.getReplay());
            }
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization",
            DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization",
            DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption("Drive SysId (Quasistatic Forward)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Quasistatic Reverse)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)",
            drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)",
            drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings()
    {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()));

        // Lock to 0° when A button is held
        controller
            .a()
            .whileTrue(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -controller.getLeftY(),
                    () -> -controller.getLeftX(),
                    () -> new Rotation2d()));

        // Switch to X pattern when X button is pressed
        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro to 0° when B button is pressed
        controller
            .b()
            .onTrue(
                Commands.runOnce(
                    () -> drive.setPose(
                        new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                    .ignoringDisable(true));

        // Pathfind to Pose when the Y button is pressed
        controller.y().onTrue(
            DriveCommands.pathFindToPose(() -> drive.getPose(), new Pose2d(3, 3, Rotation2d.kZero),
                PathConstants.ON_THE_FLY_PATH_CONSTRAINTS, 0.0,
                PathConstants.PATHGENERATION_DRIVE_TOLERANCE));

        // On-the-fly path with waypoints while the Right Bumper is held
        controller.rightBumper().whileTrue(
            new OnTheFlyPathCommand(drive, () -> drive.getPose(), new ArrayList<>(Arrays.asList()), // List
                                                                                                    // of
                                                                                                    // waypoints
                new Pose2d(6, 6, Rotation2d.k180deg), PathConstants.ON_THE_FLY_PATH_CONSTRAINTS,
                0.0, false, PathConstants.PATHGENERATION_DRIVE_TOLERANCE,
                PathConstants.PATHGENERATION_ROT_TOLERANCE_DEGREES));

        SmartDashboard.putData("Rotary: Set STOW",
            Commands.runOnce(() -> rotary.setState(RotarySubsystem.State.STOW), rotary));

        SmartDashboard.putData("Rotary: Set Raised",
            Commands.runOnce(() -> rotary.setState(RotarySubsystem.State.RAISED), rotary));

        SmartDashboard.putData("Rotary: Set Current 30", rotary.runCurrent());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return autoChooser.get();
    }
}
