// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Radians;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.io.vision.VisionIO.TagObservation;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;

public class PointTo2DTarget extends Command {
    private final RobotState robotState = RobotState.getInstance();

    private final Drive drive;
    private final PIDController rotationController = new PIDController(1, 0, 0);
    private TagObservation targetTag;
    private boolean isFinished = false;

    public PointTo2DTarget(Drive drive) {
        addRequirements(drive);
        this.drive = drive;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (robotState.getClosestTagObservation().isPresent()) {
            this.targetTag = robotState.getClosestTagObservation().get();
            rotationController.reset();
            rotationController.setSetpoint(0);

            Logger.recordOutput("AlignCommand/Rotation Setpoint", rotationController.getSetpoint());
        } else {
            this.cancel();
        }

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (robotState.getClosestTagObservation().isPresent()) {
            this.targetTag = robotState.getClosestTagObservation().get();
            double rotationOutput =
                rotationController.calculate(targetTag.yaw().in(Radians));

            Logger.recordOutput("AlignCommand/Rotation Output", rotationOutput);
            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    0.0,
                    0.0,
                    rotationOutput,
                    robotState.getEstimatedPose().getRotation()));
        } else {
            this.cancel();
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
