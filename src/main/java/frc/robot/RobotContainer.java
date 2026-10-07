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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.lib.autos.Auto;
import frc.lib.util.CommandXboxControllerExtended;
import frc.lib.util.LoggedDashboardChooser;
import frc.robot.autos.AutoFactory;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelConstants;
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
    public final Flywheel flywheel;
    public final AutoFactory autoFactory;

    // Controller
    private final CommandXboxControllerExtended controller = new CommandXboxControllerExtended(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Auto> autoChooser;
    private final LoggedDashboardChooser<Boolean> conditionalChooser;
    public static Field2d autoPreviewField = new Field2d();

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        drive = DriveConstants.get();
        VisionConstants.create();
        autoFactory = new AutoFactory(drive);
        flywheel = new Flywheel(FlywheelConstants.get());

        conditionalChooser = new LoggedDashboardChooser<>("Conditional Choice");
        conditionalChooser.addOption("True", true);
        conditionalChooser.addOption("False", false);

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices");

        autoChooser.addDefaultOption(autoFactory.noneAuto().getName(), autoFactory.noneAuto());

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX()));

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

        SmartDashboard.putData("Start Flywheel", flywheel.startCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get().getCommand();
    }
}
