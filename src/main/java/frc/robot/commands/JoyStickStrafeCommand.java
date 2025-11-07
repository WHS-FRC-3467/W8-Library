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

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.LoggedTuneableProfiledPID;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class JoyStickStrafeCommand extends Command {
    Drive drive;
    DoubleSupplier xSupplier;
    Supplier<Pose2d> targetSupplier;

    Pose2d targetPose2d;
    Pose2d currentPose2d;
    Pose2d relativePose2d;
    Rotation2d targetRotation2d;

    boolean running = false;

    static final double DEADBAND = 0.1;

    LoggedTuneableProfiledPID angleController =
            new LoggedTuneableProfiledPID("angleController", 4.5, 0.0, 0.4, 8.0, 20.0);

    LoggedTuneableProfiledPID alignController =
            new LoggedTuneableProfiledPID("alignController", 4, 0.0, 0, 3.7, 4);

    public JoyStickStrafeCommand(
            Drive drive, DoubleSupplier xSupplier, Supplier<Pose2d> targetSupplier) {
        this.drive = drive;
        this.xSupplier = xSupplier;
        this.targetSupplier = targetSupplier;

        angleController.enableContinuousInput(-Math.PI, Math.PI);
        alignController.setGoal(0);

        // addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    public void initialize() {
        alignController.reset(0);
        angleController.reset(drive.getPose().getRotation().getRadians());
        targetPose2d = targetSupplier.get();

        // Logger.recordOutput("AutoAlign/Approach/Target", targetPose2d);
    }

    // Called every time the scheduler runs while the command is scheduled.

    public void execute() {
        angleController.updatePID();
        alignController.updatePID();

        running = true;
        relativePose2d = drive.getPose().relativeTo(targetPose2d);
        targetRotation2d = targetPose2d.getRotation();

        // Calculate lateral linear velocity
        Translation2d offsetVector =
                new Translation2d(0, alignController.calculate(relativePose2d.getX()));

        // Calculate total linear velocity
        Translation2d linearVelocity =
                getLinearVelocityFromJoysticks(-xSupplier.getAsDouble(), 0)
                        .times(drive.getMaxLinearSpeedMetersPerSec())
                        .plus(offsetVector)
                        .rotateBy(targetRotation2d);

        // Calculate angular speed
        double omega =
                angleController.calculate(
                        drive.getRotation().getRadians(),
                        targetRotation2d.rotateBy(Rotation2d.k180deg).getRadians());

        // Convert to field relative speeds & send command
        ChassisSpeeds speeds =
                new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), omega);

        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
    }

    // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
        running = false;
    }

    // Returns true when the command should end.

    public boolean isFinished() {
        return false;
    }

    // Returns true when withing a lateral tolerance
    public boolean withinTolerance(double dist) {
        return running ? Math.abs(relativePose2d.getX()) < dist : false;
    }

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }
}
