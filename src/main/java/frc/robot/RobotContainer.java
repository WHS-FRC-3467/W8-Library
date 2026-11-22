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

import frc.lib.util.GamePieceVisualizer;
import frc.lib.util.PowerProfiler;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.intake.IntakeLinearConstants;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.intake.IntakeRollerConstants;
import frc.robot.subsystems.intake.IntakeSlide;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.FlywheelConstants;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.HoodConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.tower.TowerConstants;
import frc.robot.subsystems.vision.VisionConstants;
import org.wpilib.command3.button.CommandXboxController;
import org.wpilib.driverstation.GenericHID;
import org.wpilib.driverstation.XboxController;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
    private final GlobalPoseEstimator robotState = GlobalPoseEstimator.getInstance();
    private final PowerProfiler powerProfiler = PowerProfiler.getInstance();

    // Subsystems
    public final Drive drive;
    private final IntakeSlide intakeSlide;
    private final IntakeRoller intakeRoller;
    private final Indexer indexer;
    private final Tower tower;
    private final Flywheel flywheel;
    private final Hood hood;

    // Coordinates every mechanism above. Bindings and autos talk to this, not to the mechanisms.
    private final Superstructure superstructure;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        drive = DriveConstants.get();
        intakeSlide = new IntakeSlide(IntakeLinearConstants.getMechanism());
        intakeRoller = new IntakeRoller(IntakeRollerConstants.getMechanism());
        indexer = IndexerConstants.get();
        tower = TowerConstants.get();
        flywheel = new Flywheel(FlywheelConstants.get());
        hood = new Hood(HoodConstants.get());
        superstructure = new Superstructure(intakeSlide, intakeRoller, indexer, tower, flywheel, hood);
        VisionConstants.create();

        // Configure the button bindings
        configureButtonBindings();
        // Register mechanisms and generic power channels with the power profiler for logging
        registerMechanismsAndGenerics();

        GamePieceVisualizer algae =
                new GamePieceVisualizer("Algae", new Pose3d(new Translation3d(3, 3, 1), new Rotation3d(0, 0, 0)));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * org.wpilib.driverstation.Joystick} or {@link XboxController}), and then passing it to a
     * {@link org.wpilib.command2.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()));

        // Buttons only request robot states. The superstructure decides what each mechanism does.
        controller.leftTrigger().onTrue(superstructure.intake()).onFalse(superstructure.stow());
        controller.rightTrigger().onTrue(superstructure.score()).onFalse(superstructure.stow());
        controller.b().onTrue(superstructure.eject()).onFalse(superstructure.stow());

        // An override command: takes the slide from its state machine while the button is held.
        controller.menu().whileTrue(intakeSlide.home());
    }

    /** Register all of the subsystems' mechanisms and generic power channels. */
    private void registerMechanismsAndGenerics() {
        powerProfiler.setLoopPeriod(Constants.LOOP_PERIOD);
        drive.registerModules(powerProfiler);
        intakeSlide.registerMechanisms(powerProfiler);
        intakeRoller.registerMechanisms(powerProfiler);
        indexer.registerMechanisms(powerProfiler);
        tower.registerMechanisms(powerProfiler);
        flywheel.registerMechanisms(powerProfiler);
        hood.registerMechanisms(powerProfiler);
    }
}
