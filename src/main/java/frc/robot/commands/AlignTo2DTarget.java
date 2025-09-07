// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.io.vision.VisionIO.TagObservation;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class AlignTo2DTarget extends Command {
    private final Drive drive;
    private final Vision vision;
    private final PIDController strafeController = new PIDController(0.3, 0, 0);
    private final PIDController rotationController = new PIDController(1, 0, 0);
    private TagObservation targetTag;
    private boolean isFinished = false;

    public AlignTo2DTarget(Drive drive, Vision vision, DoubleSupplier ySupplier)
    {
        addRequirements(drive);
        this.drive = drive;
        this.vision = vision;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        if (vision.getClosestTagObservation().isPresent()) {
            this.targetTag = vision.getClosestTagObservation().get();
            strafeController.reset();
            rotationController.reset();
            rotationController.enableContinuousInput(-Math.PI, Math.PI);
            strafeController.setSetpoint(0);
            rotationController.setSetpoint(
                FieldConstants.aprilTagLayout
                    .getTagPose(vision.getClosestTagObservation().get().id())
                    .get().getRotation().getZ() + Math.PI);

            Logger.recordOutput("AlignCommand/Strafe Setpoint", strafeController.getSetpoint());
            Logger.recordOutput("AlignCommand/Rotation Setpoint", rotationController.getSetpoint());
        } else {
            this.cancel();
        }

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        if (vision.getClosestTagObservation().isPresent()) {
            this.targetTag = vision.getClosestTagObservation().get();
            double strafeOutput = strafeController.calculate(targetTag.yaw());
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
        } else {
            this.cancel();
        }

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
