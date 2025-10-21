// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.LoggedTuneableProfiledPID;
import frc.robot.subsystems.drive.Drive;

public abstract class AlignToPoseBase extends Command {

    private final Drive drive;
    private final Supplier<Pose2d> targetPose;
    private final DoubleSupplier joystickInput;
    private LoggedTuneableProfiledPID linearController;
    private LoggedTuneableProfiledPID angularController;

    public enum AlignMode {
        APPROACH, // Driver controls forward/backward movement while aligning
        STRAFE // Driver controls left/right movement while aligning
    }

    private final AlignMode mode;

    public AlignToPoseBase(
        Drive drive,
        Supplier<Pose2d> targetPose,
        AlignMode mode,
        DoubleSupplier joystickInput,
        LoggedTuneableProfiledPID linearController,
        LoggedTuneableProfiledPID angularController)
    {
        this.drive = drive;
        this.targetPose = targetPose;
        this.mode = mode;
        this.joystickInput = joystickInput;
        this.linearController = linearController;
        this.angularController = angularController;

        angularController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        ChassisSpeeds fieldVelocity =
            ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());

        linearController.reset(0.0);

        angularController.reset(
            drive.getRotation().getRadians(),
            fieldVelocity.omegaRadiansPerSecond);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        // Checks if tunable values for PID have changed and updates them if so
        linearController.updatePID();
        angularController.updatePID();

        var relativePose2d = drive.getPose().relativeTo(targetPose.get());
        var targetRotation2d = targetPose.get().getRotation();
        var linearVelocity = new Translation2d();
        var offsetVector = new Translation2d();

        switch (mode) {
            case STRAFE:
                offsetVector =
                    new Translation2d(linearController.calculate(relativePose2d.getX()), 0);

                // Calculate total linear velocity
                linearVelocity =
                    getLinearVelocityFromJoysticks(0,
                        -joystickInput.getAsDouble()).times(drive.getMaxLinearSpeedMetersPerSec())
                            .plus(offsetVector)
                            .rotateBy(targetRotation2d);
                break;

            case APPROACH:
                offsetVector =
                    new Translation2d(0, linearController.calculate(relativePose2d.getY()));

                // Calculate total linear velocity
                linearVelocity =
                    getLinearVelocityFromJoysticks(-joystickInput.getAsDouble(),
                        0).times(drive.getMaxLinearSpeedMetersPerSec())
                            .plus(offsetVector)
                            .rotateBy(targetRotation2d);
                break;

            default:
                break;
        }

        double angularOutput = angularController.calculate(
            drive.getRotation().getRadians(),
            targetPose.get().getRotation().getRadians());

        // Convert to field relative speeds & send command
        ChassisSpeeds speeds =
            new ChassisSpeeds(
                linearVelocity.getX(),
                linearVelocity.getY(),
                angularOutput);

        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
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

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y)
    {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), 0.1); // TODO: figure out
                                                                                // deadband
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();
    }
}
