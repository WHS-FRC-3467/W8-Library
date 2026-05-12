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

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.lib.commands.SteppableCommandGroup;
import frc.lib.util.CommandXboxControllerExtended;
import frc.lib.util.GamePieceVisualizer;
import frc.lib.util.LoggedDashboardChooser;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.lasercan1.LaserCAN1;
import frc.robot.subsystems.lasercan1.LaserCAN1Constants;
import frc.robot.subsystems.vision.VisionConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
    private final RobotState robotState = RobotState.getInstance();

    // Subsystems
    public final Drive drive;
    private final LaserCAN1 laserCAN1;
    private final Arm arm;
    // Controller
    private final CommandXboxControllerExtended controller = new CommandXboxControllerExtended(0);

    // Dashboard inputs
    // private final LoggedDashboardChooser<AutoCommand> autoChooser;
    private final LoggedDashboardChooser<Boolean> conditionalChooser;
    public static Field2d autoPreviewField = new Field2d();

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        drive = DriveConstants.get();
        laserCAN1 = LaserCAN1Constants.get();
        arm = ArmConstants.get();

        VisionConstants.create();

        conditionalChooser = new LoggedDashboardChooser<>("Conditional Choice");
        conditionalChooser.addOption("True", true);
        conditionalChooser.addOption("False", false);

        // Set up auto routines
        // autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        // SmartDashboard.putData("Auto Preview", autoPreviewField);

        // autoChooser.addDefaultOption("None", new NoneAuto());
        // autoChooser.addOption("ExampleAuto", new ExampleAuto(drive));
        // autoChooser.addOption("BranchingAuto",
        // new BranchingAuto(drive, () -> conditionalChooser.get()));

        // autoChooser.onChange(auto -> {
        // autoPreviewField.getObject("path").setPoses(auto.getAllPathPoses());
        // });

        // autoChooser.addOption("Drive Wheel Radius Characterization",
        // new WheelCharacterizationAuto(drive));

        // autoChooser.addOption("Wheel Slip Characterization", new
        // WheelSlipAuto(drive));

        // Configure the button bindings
        configureButtonBindings();

        GamePieceVisualizer algae =
                new GamePieceVisualizer(
                        "Algae", new Pose3d(new Translation3d(3, 3, 1), new Rotation3d(0, 0, 0)));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        controller.povUp().onTrue(arm.moveUpperBy(RadiansPerSecond.of(5.0), true));
        controller.povDown().onTrue(arm.moveUpperBy(RadiansPerSecond.of(-5.0), false));
        controller.povRight().onTrue(arm.moveLowerBy(RadiansPerSecond.of(5.0), true));
        controller.povLeft().onTrue(arm.moveLowerBy(RadiansPerSecond.of(-5.0), false));

        controller.a().onTrue(arm.stopUpper());
        controller.b().onTrue(arm.stopLower());

        // Default command, normal field-relative drive

        // Lock to 0° when A button is held
        // controller
        // .a()
        // .whileTrue(
        // DriveCommands.joystickDriveAtAngle(
        // drive,
        // () -> -controller.getLeftY(),
        // () -> -controller.getLeftX(),
        // () -> new Rotation2d()));

        // Switch to X pattern when X button is pressed
        // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro to 0° when B button is pressed
        controller
                .b()
                .onTrue(
                        Commands.runOnce(
                                        () ->
                                                robotState.resetPose(
                                                        new Pose2d(
                                                                robotState
                                                                        .getEstimatedPose()
                                                                        .getTranslation(),
                                                                new Rotation2d())))
                                .ignoringDisable(true));

        // SmartDashboard.putData("Superstructure: Stow",
        // superstructure.setGoal(Superstructure.Setpoint.STOW));
        // SmartDashboard.putData("Superstructure: Raised",
        // superstructure.setGoal(Superstructure.Setpoint.RAISED));

        Command steppableCommand =
                new SteppableCommandGroup(
                        controller.x(),
                        controller.y(),
                        Commands.runOnce(() -> System.out.println("Step 1")),
                        Commands.runOnce(() -> System.out.println("Step 2")),
                        Commands.runOnce(() -> System.out.println("Step 3")));

        SmartDashboard.putData("Steppable Command", steppableCommand);

        // controller.x()
        // .whileTrue(new DriveToPose(drive, () -> new Pose2d(5, 5,
        // Rotation2d.fromDegrees(90)))
        // .withTolerance(Inches.of(3), Degrees.of(5)));

        // controller.x()
        // .whileTrue(new AlignToPose(drive, () -> new Pose2d(5, 5,
        // Rotation2d.fromDegrees(0)),
        // AlignMode.STRAFE, () -> controller.getRightX()));

        // Right bumper: Shoot on the Move
        // controller.rightBumper().whileTrue(
        // turret.shoot(drive, () -> -controller.getLeftX(), () ->
        // -controller.getLeftY()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return autoChooser.get();
        return Commands.none();
    }

    /** This function is called periodically by Robot.java when disabled. */
    public void checkStartPose() {

        // /* Starting pose checker for auto */
        // autoPreviewField.setRobotPose(robotState.getEstimatedPose());

        // try {
        // double distanceFromStartPose = robotState.getEstimatedPose().getTranslation()
        // .getDistance(autoPreviewField.getObject("path").getPoses().get(0).getTranslation());
        // double degreesFromStartPose =
        // Math.abs(robotState.getEstimatedPose().getRotation()
        // .minus(
        // autoPreviewField.getObject("path").getPoses().get(0).getRotation())
        // .getDegrees());

        // SmartDashboard.putNumber("Auto Pose Check/Inches from Start",
        // Math.round(distanceFromStartPose * 100.0) / 100.0);
        // SmartDashboard.putBoolean(
        // "Auto Pose Check/Robot Position within "
        // + PathConstants.STARTING_POSE_DRIVE_TOLERANCE.in(Inches) + " inches",
        // distanceFromStartPose <
        // PathConstants.STARTING_POSE_DRIVE_TOLERANCE.in(Inches));
        // SmartDashboard.putNumber("Auto Pose Check/Degrees from Start",
        // Math.round(degreesFromStartPose * 100.0) / 100.0);
        // SmartDashboard.putBoolean(
        // "Auto Pose Check/Robot Rotation within "
        // + PathConstants.STARTING_POSE_ROT_TOLERANCE_DEGREES + " degrees",
        // degreesFromStartPose < PathConstants.STARTING_POSE_ROT_TOLERANCE_DEGREES
        // .in(Degrees));

        // } catch (Exception e) {
        // SmartDashboard.putNumber("Auto Pose Check/Inches from Start", -1);
        // SmartDashboard.putBoolean(
        // "Auto Pose Check/Robot Position within "
        // + PathConstants.STARTING_POSE_DRIVE_TOLERANCE.in(Inches) + " inches",
        // false);
        // SmartDashboard.putNumber("Auto Pose Check/Degrees from Start", -1);
        // SmartDashboard.putBoolean(
        // "Auto Pose Check/Robot Rotation within "
        // + PathConstants.STARTING_POSE_ROT_TOLERANCE_DEGREES.in(Degrees) + " degrees",
        // false);

    }
}
