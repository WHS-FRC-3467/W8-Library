/*
 * Copyright (C) 2026 Windham Windup
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

import frc.lib.util.LoggedTunableNumber;
import frc.robot.GlobalPoseEstimator;
import frc.robot.subsystems.drive.Drive;

import lombok.Getter;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.Units;
import org.wpilib.system.Timer;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Factory class for creating drive-related commands.
 *
 * <p>Provides factory methods for common drive operations including:
 *
 * <ul>
 *   <li>Joystick-controlled driving (field-relative and robot-relative)
 *   <li>Angle locking for driver assistance
 *   <li>Pathfinding to specific field positions
 *   <li>System identification and characterization
 * </ul>
 *
 * <p>All commands are designed to work with the {@link Drive} subsystem and integrate with
 * PathPlanner for autonomous path following.
 */
public class DriveCommands {
    @Getter private static final double ANGLE_MAX_VELOCITY = 8.3;
    @Getter private static final double ANGLE_MAX_ACCELERATION = 20.0;
    private static final LoggedTunableNumber ANGLE_KP =
            new LoggedTunableNumber("Drive/AngleP", 9.0);
    private static final LoggedTunableNumber ANGLE_KD =
            new LoggedTunableNumber("Drive/AngleD", 0.6);
    private static final LoggedTunableNumber ANGLE_TOLERANCE_DEGREES =
            new LoggedTunableNumber("Drive/AngleToleranceDegrees", 2.5d);
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 2.0; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.2; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.01; // Rad/Sec^2

    private DriveCommands() {}

    /**
     * Converts joystick inputs to a linear velocity vector.
     *
     * @param x the x-axis joystick input
     * @param y the y-axis joystick input
     * @return the calculated linear velocity as a Translation2d
     */
    public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        double linearMagnitude = Math.pow(Math.hypot(x, y), 2);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }

    public static Command stopWithX(Drive drive) {
        return Command.requiring(drive)
                .executing(coroutine -> drive.stopWithX())
                .named("Stop With X");
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     *
     * @param drive the drive subsystem
     * @param xSupplier supplier for x-axis joystick input
     * @param ySupplier supplier for y-axis joystick input
     * @param omegaSupplier supplier for rotation joystick input
     * @return the joystick drive command
     */
    public static Command joystickDrive(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        GlobalPoseEstimator robotState = GlobalPoseEstimator.getInstance();
        return Command.requiring(drive)
                .executing(
                        coroutine -> {
                            while (true) {
                                // Get linear velocity
                                Translation2d linearVelocity =
                                        getLinearVelocityFromJoysticks(
                                                xSupplier.getAsDouble(), ySupplier.getAsDouble());

                                // Apply rotation deadband
                                double omega = omegaSupplier.getAsDouble();

                                // Square rotation value for more precise control
                                omega = Math.copySign(omega * omega, omega);

                                // Convert to field relative speeds & send command
                                ChassisVelocities speeds =
                                        new ChassisVelocities(
                                                linearVelocity.getX()
                                                        * drive.getMaxLinearSpeedMetersPerSec(),
                                                linearVelocity.getY()
                                                        * drive.getMaxLinearSpeedMetersPerSec(),
                                                omega * drive.getMaxAngularSpeedRadPerSec());
                                boolean isFlipped =
                                        MatchState.getAlliance().isPresent()
                                                && MatchState.getAlliance().get() == Alliance.RED;
                                drive.runVelocity(
                                        speeds.toRobotRelative(
                                                isFlipped
                                                        ? robotState
                                                                .getEstimatedPose()
                                                                .getRotation()
                                                                .plus(new Rotation2d(Math.PI))
                                                        : robotState
                                                                .getEstimatedPose()
                                                                .getRotation()));
                                coroutine.yield();
                            }
                        })
                .named("Joystick Drive");
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     *
     * @param drive the drive subsystem
     * @param xSupplier supplier for x-axis joystick input
     * @param ySupplier supplier for y-axis joystick input
     * @param rotationSupplier supplier for target rotation angle
     * @return the joystick drive at angle command
     */
    public static Command joystickDriveAtAngle(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Rotation2d> rotationSupplier,
            boolean stopWithX) {
        GlobalPoseEstimator robotState = GlobalPoseEstimator.getInstance();

        // Create PID controller
        ProfiledPIDController angleController =
                new ProfiledPIDController(
                        ANGLE_KP.get(),
                        0.0,
                        ANGLE_KD.get(),
                        new TrapezoidProfile.Constraints(
                                ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.setTolerance(Units.degreesToRadians(ANGLE_TOLERANCE_DEGREES.get()));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Command.requiring(drive)
                .executing(
                        coroutine -> {
                            // Reset PID controller when command starts
                            angleController.reset(
                                    robotState.getEstimatedPose().getRotation().getRadians());
                            angleController.setP(ANGLE_KP.get());
                            angleController.setD(ANGLE_KD.get());
                            angleController.setTolerance(
                                    Units.degreesToRadians(ANGLE_TOLERANCE_DEGREES.get()));
                            while (true) {
                                // Get linear velocity
                                Translation2d linearVelocity =
                                        getLinearVelocityFromJoysticks(
                                                xSupplier.getAsDouble(), ySupplier.getAsDouble());

                                // Calculate angular speed
                                double omega =
                                        angleController.calculate(
                                                robotState
                                                        .getEstimatedPose()
                                                        .getRotation()
                                                        .getRadians(),
                                                rotationSupplier.get().getRadians());

                                Logger.recordOutput("AngleAtGoal", angleController.atGoal());
                                if (angleController.atSetpoint()) {
                                    if (Math.hypot(linearVelocity.getX(), linearVelocity.getY())
                                                    < 1e-2
                                            && stopWithX) {
                                        // If we're at the angle goal and
                                        // the driver isn't commanding any
                                        // linear motion, stop the robot to prevent drifting.
                                        drive.stopWithX();
                                        coroutine.yield();
                                        continue;
                                    } else {
                                        // If we're at the angle goal but the driver is commanding
                                        // linear
                                        // motion, set angular speed to zero to prevent
                                        // overshooting.
                                        omega = 0.0;
                                    }
                                }

                                // Convert to field relative speeds & send command
                                ChassisVelocities speeds =
                                        new ChassisVelocities(
                                                linearVelocity.getX()
                                                        * drive.getMaxLinearSpeedMetersPerSec(),
                                                linearVelocity.getY()
                                                        * drive.getMaxLinearSpeedMetersPerSec(),
                                                omega);
                                boolean isFlipped =
                                        MatchState.getAlliance().isPresent()
                                                && MatchState.getAlliance().get() == Alliance.RED;
                                drive.runVelocity(
                                        speeds.toRobotRelative(
                                                isFlipped
                                                        ? robotState
                                                                .getEstimatedPose()
                                                                .getRotation()
                                                                .plus(new Rotation2d(Math.PI))
                                                        : robotState
                                                                .getEstimatedPose()
                                                                .getRotation()));
                                coroutine.yield();
                            }
                        })
                .named("Joystick Drive At Angle");
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>This command should only be used in voltage control mode.
     *
     * @param drive the drive subsystem
     * @return the feedforward characterization command
     */
    public static Command feedforwardCharacterization(Drive drive) {
        List<Double> velocitySamples = new ArrayList<>();
        List<Double> voltageSamples = new ArrayList<>();
        Timer timer = new Timer();

        boolean[] collecting = {false};
        return Command.requiring(drive)
                .executing(
                        coroutine -> {
                            collecting[0] = false;
                            velocitySamples.clear();
                            voltageSamples.clear();
                            timer.restart();
                            while (timer.get() < FF_START_DELAY) {
                                drive.runCharacterization(0.0);
                                coroutine.yield();
                            }
                            timer.restart();
                            collecting[0] = true;
                            while (true) {
                                double voltage = timer.get() * FF_RAMP_RATE;
                                drive.runCharacterization(voltage);
                                velocitySamples.add(drive.getFFCharacterizationVelocity());
                                voltageSamples.add(voltage);
                                coroutine.yield();
                            }
                        })
                .whenCanceled(
                        () -> {
                            if (!collecting[0]) {
                                return;
                            }
                            int n = velocitySamples.size();
                            double sumX = 0.0;
                            double sumY = 0.0;
                            double sumXY = 0.0;
                            double sumX2 = 0.0;
                            for (int i = 0; i < n; i++) {
                                sumX += velocitySamples.get(i);
                                sumY += voltageSamples.get(i);
                                sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                            }
                            double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                            double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                            NumberFormat formatter = new DecimalFormat("#0.00000");
                            String results =
                                    "Drive FF Characterization Results - "
                                            + "kS: "
                                            + formatter.format(kS)
                                            + ", kV: "
                                            + formatter.format(kV);
                            DriverStationErrors.reportWarning(results, false);
                        })
                .named("Feedforward Characterization");
    }

    /**
     * Measures the robot's wheel radius by spinning in a circle.
     *
     * @param drive the drive subsystem
     * @return the wheel radius characterization command
     */
    public static Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        Timer timer = new Timer();
        return Command.requiring(drive)
                .executing(
                        coroutine -> {
                            limiter.reset(0.0);
                            state.measurementStarted = false;
                            timer.restart();
                            while (true) {
                                double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                drive.runVelocity(new ChassisVelocities(0.0, 0.0, speed));
                                if (!state.measurementStarted && timer.get() >= 1.0) {
                                    state.positions =
                                            drive.getWheelRadiusCharacterizationPositions();
                                    state.lastAngle = drive.getRawGyroAngle();
                                    state.gyroDelta = 0.0;
                                    state.measurementStarted = true;
                                }
                                if (state.measurementStarted) {
                                    var rotation = drive.getRawGyroAngle();
                                    state.gyroDelta +=
                                            Math.abs(rotation.minus(state.lastAngle).getRadians());
                                    state.lastAngle = rotation;
                                }
                                coroutine.yield();
                            }
                        })
                .whenCanceled(
                        () -> {
                            if (!state.measurementStarted) {
                                return;
                            }
                            double[] positions = drive.getWheelRadiusCharacterizationPositions();
                            double wheelDelta = 0.0;
                            for (int i = 0; i < 4; i++) {
                                double thisWheelDelta = Math.abs(positions[i] - state.positions[i]);
                                thisWheelDelta =
                                        (state.gyroDelta * Drive.DRIVE_BASE_RADIUS)
                                                / thisWheelDelta;
                                System.out.println(i + " " + thisWheelDelta);

                                wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                            }

                            double wheelRadius =
                                    (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                            NumberFormat formatter = new DecimalFormat("#0.000");
                            String results =
                                    "Wheel Radius Characterization Results"
                                            + " - Wheel Delta: "
                                            + formatter.format(wheelDelta)
                                            + " radians, "
                                            + "Gyro Delta: "
                                            + formatter.format(state.gyroDelta)
                                            + " radians, "
                                            + "Wheel Radius: "
                                            + formatter.format(wheelRadius)
                                            + " meters ("
                                            + formatter.format(Units.metersToInches(wheelRadius))
                                            + " inches)";
                            DriverStationErrors.reportWarning(results, false);
                        })
                .named("Wheel Radius Characterization");
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
        boolean measurementStarted = false;
    }
}
