// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class AlignTo2DTarget extends Command {
    private final Drive drive;
    private final Supplier<Pose2d> relativePose;
    private final PIDController strafeController = new PIDController(0.3, 0, 0);
    private final PIDController rotationController = new PIDController(1, 0, 0);

    private boolean isFinished = false;

    public AlignTo2DTarget(Drive drive, Supplier<Pose2d> relativePose)
    {
        addRequirements(drive);
        this.drive = drive;
        this.relativePose = relativePose;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {

        strafeController.reset();
        rotationController.reset();
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        strafeController.setSetpoint(0);
        rotationController
            .setSetpoint(relativePose.get().getRotation().getRadians());

        Logger.recordOutput("AlignCommand/Strafe Setpoint", strafeController.getSetpoint());
        Logger.recordOutput("AlignCommand/Rotation Setpoint", rotationController.getSetpoint());

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {

        double strafeOutput = strafeController.calculate(relativePose.get().getY());
        double rotationOutput =
            rotationController.calculate(drive.getPose().getRotation().getRadians());

        Logger.recordOutput("AlignCommand/Strafe Output", strafeOutput);
        Logger.recordOutput("AlignCommand/Rotation Output", rotationOutput);
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                0.0,
                strafeOutput,
                rotationOutput,
                drive.getPose().getRotation()));

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return isFinished;
    }
}
