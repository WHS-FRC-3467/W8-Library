// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.commands.DriveToPoseBase;
import frc.lib.util.LoggedTuneableProfiledPID;
import frc.robot.subsystems.drive.Drive;

public class DriveToPose extends DriveToPoseBase {

    private final LoggedTuneableProfiledPID linearController =
        new LoggedTuneableProfiledPID("DriveToPose/LinearController", 3.0, 0, 0.1, 0, 3.0);
    private final LoggedTuneableProfiledPID angularController =
        new LoggedTuneableProfiledPID("DriveToPose/AngularController", 3.0, 0, 0, 0, 0);

    public DriveToPose(Drive drive, Supplier<Pose2d> targetPose)
    {
        super(drive, targetPose);
    }

    // Called when the command is initially scheduled.
    public void initialize()
    {
        withLinearPID(linearController);
        withAngularPID(angularController);
        super.initialize();

    }

    public void execute()
    {
        super.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {}
}
