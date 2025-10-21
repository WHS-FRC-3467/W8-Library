// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggedTuneableProfiledPID;
import frc.robot.subsystems.drive.Drive; // TODO: refactor drive to exist in lib

public class DriveToPoseBase extends Command {
    private final Drive drive;
    private final Supplier<Pose2d> targetPose;

    private LoggedTuneableProfiledPID linearController;
    private LoggedTuneableProfiledPID angularController;

    private LoggedTunableNumber maxLinearSpeed;
    private LoggedTunableNumber maxAngularSpeed;

    private Optional<Double> distanceTolerance = Optional.empty();
    private Optional<Double> angleTolerance = Optional.empty();


    public DriveToPoseBase(
        Drive drive,
        Supplier<Pose2d> targetPose,
        LoggedTuneableProfiledPID linearController,
        LoggedTuneableProfiledPID angularController,
        LoggedTunableNumber maxLinearSpeed,
        LoggedTunableNumber maxAngularSpeed)
    {
        this.drive = drive;
        this.targetPose = targetPose;
        this.linearController = linearController;
        this.angularController = angularController;
        this.maxLinearSpeed = maxLinearSpeed;
        this.maxAngularSpeed = maxAngularSpeed;

        angularController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drive);
    }

    public DriveToPoseBase withLinearPID(LoggedTuneableProfiledPID pid)
    {
        this.linearController = pid;
        return this;
    }

    public DriveToPoseBase withAngularPID(LoggedTuneableProfiledPID pid)
    {
        this.angularController = pid;
        angularController.enableContinuousInput(-Math.PI, Math.PI);
        return this;
    }

    public DriveToPoseBase withDistanceTolerance(Distance tolerance)
    {
        distanceTolerance = Optional.of(tolerance.in(Meters));
        return this;
    }

    public DriveToPoseBase withAngularTolerance(Angle tolerance)
    {
        angleTolerance = Optional.of(tolerance.in(Radians));
        return this;
    }

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

        // Calculate translation and direction to target
        Translation2d translationToTarget =
            targetPose.get().getTranslation().minus(drive.getPose().getTranslation());

        Rotation2d directionToTarget = translationToTarget.getAngle();

        // Calculate outputs from controllers
        double linearOutput = -linearController.calculate(translationToTarget.getNorm());

        linearOutput = MathUtil.clamp(
            linearOutput,
            -maxLinearSpeed.get(),
            maxLinearSpeed.get());

        double angularOutput = angularController.calculate(
            drive.getRotation().getRadians(),
            targetPose.get().getRotation().getRadians());

        angularOutput = MathUtil.clamp(
            angularOutput,
            -maxAngularSpeed.get(),
            maxAngularSpeed.get());

        // Convert to robot-relative speeds and set request velocities
        var fieldRelativeSpeed = new ChassisSpeeds(
            linearOutput * Math.cos(directionToTarget.getRadians()),
            linearOutput * Math.sin(directionToTarget.getRadians()),
            angularOutput);

        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeed, drive.getRotation()));

        Logger.recordOutput("DriveToPose/Target Pose", targetPose.get());
        Logger.recordOutput("DriveToPose/Distance To Target (m)", translationToTarget.getNorm());
        Logger.recordOutput("DriveToPose/Angle To Target (deg)", directionToTarget.getDegrees());
        Logger.recordOutput("DriveToPose/LinearController/Error",
            linearController.getPositionError());
        Logger.recordOutput("DriveToPose/LinearController/Output", linearOutput);
        Logger.recordOutput("DriveToPose/AngularController/Error",
            angularController.getPositionError());
        Logger.recordOutput("DriveToPose/AngularController/Output", angularOutput);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        boolean withinDistanceTolerance = distanceTolerance
            .map(tolerance -> Math.abs(linearController.getPositionError()) < tolerance)
            .orElse(false);

        boolean withinAngularTolerance = angleTolerance
            .map(tolerance -> Math.abs(angularController.getPositionError()) < tolerance)
            .orElse(false);

        Logger.recordOutput("DriveToPose/Distance Tolerance Present",
            distanceTolerance.isPresent());
        Logger.recordOutput("DriveToPose/Within Distance Tolerance", withinDistanceTolerance);
        Logger.recordOutput("DriveToPose/Angular Tolerance Present",
            angleTolerance.isPresent());
        Logger.recordOutput("DriveToPose/Within Angular Tolerance", withinAngularTolerance);



        if (distanceTolerance.isPresent() && angleTolerance.isPresent()) {
            return withinDistanceTolerance && withinAngularTolerance;
        } else if (distanceTolerance.isPresent()) {
            return withinDistanceTolerance;
        } else if (angleTolerance.isPresent()) {
            return withinAngularTolerance;
        } else {
            return false;
        }
    }
}
