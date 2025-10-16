// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.LoggedTuneableProfiledPID;
import frc.robot.subsystems.drive.Drive;

public class DriveToPose extends Command {
    private final Drive drive;
    private final Supplier<Pose2d> targetPose;
    private final LoggedTuneableProfiledPID linearController;
    private final LoggedTuneableProfiledPID angularController;

    public DriveToPose(Drive drive, Supplier<Pose2d> targetPose)
    {
        this.drive = drive;
        this.targetPose = targetPose;
        this.linearController =
            new LoggedTuneableProfiledPID("DriveToPose/LinearController", 3.0, 0, 0.1, 3, 3);
        this.angularController =
            new LoggedTuneableProfiledPID("DriveToPose/AngularController", 4.0, 0, 0, 0, 0);
        angularController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        ChassisSpeeds fieldVelocity =
            ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());
        Translation2d linearFieldVelocity =
            new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);

        linearController.reset(
            drive.getPose().getTranslation().getDistance(targetPose.get().getTranslation()),
            Math.min(0.0,
                -linearFieldVelocity
                    .rotateBy(
                        targetPose.get().getTranslation()
                            .minus(drive.getPose().getTranslation())
                            .getAngle()
                            .unaryMinus())
                    .getX()));

        angularController.reset( // Not sure if needed
            drive.getRotation().getRadians(),
            fieldVelocity.omegaRadiansPerSecond);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        var translationToTarget =
            targetPose.get().getTranslation().minus(drive.getPose().getTranslation());
        var distanceToTarget = translationToTarget.getNorm();
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
