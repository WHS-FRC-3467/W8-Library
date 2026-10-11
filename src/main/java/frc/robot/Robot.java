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

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;

import frc.lib.util.BatterySimCurrentAccumulator;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.DriveConstants;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command3.Scheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    public Robot() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0 -> Logger.recordMetadata("GitDirty", "All changes committed");
            case 1 -> Logger.recordMetadata("GitDirty", "Uncomitted changes");
            default -> Logger.recordMetadata("GitDirty", "Unknown");
        }

        // Set up data receivers & replay source
        switch (Constants.currentMode) {
            case REAL -> {
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
            }

            case SIM -> {
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
            }

            case REPLAY -> {
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(
                        new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            }
        }

        // Start AdvantageKit logger
        Logger.start();

        // Check for valid swerve config
        var modules =
                new SwerveModuleConstants[] {
                    DriveConstants.FrontLeft,
                    DriveConstants.FrontRight,
                    DriveConstants.BackLeft,
                    DriveConstants.BackRight
                };
        for (var constants : modules) {
            if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
                    || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
                throw new RuntimeException("You are using an unsupported swerve configuration");
            }
        }

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        new RobotContainer();
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        // Optionally switch the thread to high priority to improve loop
        // timing (see the template project documentation for details)
        // Threads.setCurrentThreadPriority(true, 99);

        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        Scheduler.getDefault().run();

        // Simulate the loaded battery voltage after all subsystems have contributed their current
        // draws
        if (Constants.currentMode == Mode.SIM) {
            BatterySimCurrentAccumulator.setSimulatedBatteryLoadedVoltage();
        }

        // Return to non-RT thread priority (do not modify the first argument)
        // Threads.setCurrentThreadPriority(false, 10);
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {}

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {}

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {}

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    /** This function is called once when test mode is enabled. */
    @Override
    public void utilityInit() {}

    /** This function is called periodically during test mode. */
    @Override
    public void utilityPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
