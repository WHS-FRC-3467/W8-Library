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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.Mechanism.TunablePidConfig;
import frc.lib.posestimator.SwerveOdometry.OdometryObservation;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggerHelper;
import frc.lib.util.PID;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import frc.robot.commands.ResilientTrajectoryFollower;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class Drive extends SubsystemBase {
    public final RobotState robotState = RobotState.getInstance();

    // TunerConstants doesn't include these constants, so they are declared locally
    static final double ODOMETRY_FREQUENCY =
            new CANBus(DriveConstants.drivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;

    public static final double DRIVE_BASE_RADIUS =
            Math.max(
                    Math.max(
                            Math.hypot(
                                    DriveConstants.FrontLeft.LocationX,
                                    DriveConstants.FrontLeft.LocationY),
                            Math.hypot(
                                    DriveConstants.FrontRight.LocationX,
                                    DriveConstants.FrontRight.LocationY)),
                    Math.max(
                            Math.hypot(
                                    DriveConstants.BackLeft.LocationX,
                                    DriveConstants.BackLeft.LocationY),
                            Math.hypot(
                                    DriveConstants.BackRight.LocationX,
                                    DriveConstants.BackRight.LocationY)));

    public static final List<Translation2d> MODULE_TRANSLATIONS =
            List.of(
                    new Translation2d(
                            DriveConstants.FrontLeft.LocationX, DriveConstants.FrontLeft.LocationY),
                    new Translation2d(
                            DriveConstants.FrontRight.LocationX,
                            DriveConstants.FrontRight.LocationY),
                    new Translation2d(
                            DriveConstants.BackLeft.LocationX, DriveConstants.BackLeft.LocationY),
                    new Translation2d(
                            DriveConstants.BackRight.LocationX,
                            DriveConstants.BackRight.LocationY));

    static final Lock odometryLock = new ReentrantLock();

    private static final LoggedTunableNumber AUTO_TRANSLATION_KP =
            new LoggedTunableNumber("Drive/AutoTranslationPID/kP", 6.0);
    private static final LoggedTunableNumber AUTO_TRANSLATION_KI =
            new LoggedTunableNumber("Drive/AutoTranslationPID/kI", 0.0);
    private static final LoggedTunableNumber AUTO_TRANSLATION_KD =
            new LoggedTunableNumber("Drive/AutoTranslationPID/kD", 0.0);
    private static final LoggedTunableNumber AUTO_THETA_KP =
            new LoggedTunableNumber("Drive/AutoThetaPID/kP", 8.0);
    private static final LoggedTunableNumber AUTO_THETA_KI =
            new LoggedTunableNumber("Drive/AutoThetaPID/kI", 0.0);
    private static final LoggedTunableNumber AUTO_THETA_KD =
            new LoggedTunableNumber("Drive/AutoThetaPID/kD", 0.1);

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final Module[] modules = new Module[4]; // FL, FR, BL, BR

    private final SysIdRoutine sysId;

    private final Alert gyroDisconnectedAlert =
            new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    private SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(MODULE_TRANSLATIONS.toArray(Translation2d[]::new));

    private final TunablePidConfig driveTunablePID;
    private final TunablePidConfig steerTunablePID;

    private final PIDController autoXController =
            new PIDController(
                    AUTO_TRANSLATION_KP.get(),
                    AUTO_TRANSLATION_KI.get(),
                    AUTO_TRANSLATION_KD.get());
    private final PIDController autoYController =
            new PIDController(
                    AUTO_TRANSLATION_KP.get(),
                    AUTO_TRANSLATION_KI.get(),
                    AUTO_TRANSLATION_KD.get());
    private final PIDController autoThetaController =
            new PIDController(AUTO_THETA_KP.get(), AUTO_THETA_KI.get(), AUTO_THETA_KD.get());

    private TunablePidConfig makeTunablePID(String prefix, PID defaultPid) {
        LoggedTunableNumber kp =
                new LoggedTunableNumber("Drive/PID/" + prefix + "/kP", defaultPid.P());
        LoggedTunableNumber ki =
                new LoggedTunableNumber("Drive/PID/" + prefix + "/kI", defaultPid.I());
        LoggedTunableNumber kd =
                new LoggedTunableNumber("Drive/PID/" + prefix + "/kD", defaultPid.D());
        LoggedTunableNumber ka =
                new LoggedTunableNumber("Drive/PID/" + prefix + "/kA", defaultPid.A());
        LoggedTunableNumber kv =
                new LoggedTunableNumber("Drive/PID/" + prefix + "/kV", defaultPid.V());
        LoggedTunableNumber kg =
                new LoggedTunableNumber("Drive/PID/" + prefix + "/kG", defaultPid.G());
        LoggedTunableNumber ks =
                new LoggedTunableNumber("Drive/PID/" + prefix + "/kS", defaultPid.S());
        int id = Objects.hash(this, prefix);
        return new TunablePidConfig(PIDSlot.SLOT_0, kp, ki, kd, ka, kv, kg, ks, id);
    }

    /**
     * Constructs a new Drive subsystem.
     *
     * @param gyroIO IO interface for the gyro
     * @param flModuleIO IO interface for the front left module
     * @param frModuleIO IO interface for the front right module
     * @param blModuleIO IO interface for the back left module
     * @param brModuleIO IO interface for the back right module
     */
    public Drive(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;

        modules[0] = new Module(flModuleIO, 0, DriveConstants.FrontLeft);
        modules[1] = new Module(frModuleIO, 1, DriveConstants.FrontRight);
        modules[2] = new Module(blModuleIO, 2, DriveConstants.BackLeft);
        modules[3] = new Module(brModuleIO, 3, DriveConstants.BackRight);

        // Usage reporting for swerve template
        HAL.report(
                tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();
        // Configure SysId
        sysId =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) ->
                                        Logger.recordOutput("Drive/SysIdState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

        driveTunablePID =
                makeTunablePID(
                        "Drive",
                        new PID(SlotConfigs.from(DriveConstants.FrontLeft.DriveMotorGains)));
        steerTunablePID =
                makeTunablePID(
                        "Steer",
                        new PID(SlotConfigs.from(DriveConstants.FrontLeft.SteerMotorGains)));
        autoThetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    @SuppressWarnings("LockNotBeforeTry")
    public void periodic() {
        LoggerHelper.recordCurrentCommand("Drive", this);

        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        LoggedTunableNumber.ifChanged(
                driveTunablePID.id,
                () -> {
                    for (Module module : modules) {
                        module.setDrivePID(
                                new PID(
                                        driveTunablePID.kp.get(),
                                        driveTunablePID.ki.get(),
                                        driveTunablePID.kd.get(),
                                        driveTunablePID.ka.get(),
                                        driveTunablePID.kv.get(),
                                        driveTunablePID.kg.get(),
                                        driveTunablePID.ks.get()));
                    }
                },
                driveTunablePID.kp,
                driveTunablePID.ki,
                driveTunablePID.kd,
                driveTunablePID.ka,
                driveTunablePID.kv,
                driveTunablePID.kg,
                driveTunablePID.ks);

        LoggedTunableNumber.ifChanged(
                steerTunablePID.id,
                () -> {
                    for (Module module : modules) {
                        module.setTurnPID(
                                new PID(
                                        steerTunablePID.kp.get(),
                                        steerTunablePID.ki.get(),
                                        steerTunablePID.kd.get(),
                                        steerTunablePID.ka.get(),
                                        steerTunablePID.kv.get(),
                                        steerTunablePID.kg.get(),
                                        steerTunablePID.ks.get()));
                    }
                },
                steerTunablePID.kp,
                steerTunablePID.ki,
                steerTunablePID.kd,
                steerTunablePID.ka,
                steerTunablePID.kv,
                steerTunablePID.kg,
                steerTunablePID.ks);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    autoXController.setPID(
                            AUTO_TRANSLATION_KP.get(),
                            AUTO_TRANSLATION_KI.get(),
                            AUTO_TRANSLATION_KD.get());
                    autoYController.setPID(
                            AUTO_TRANSLATION_KP.get(),
                            AUTO_TRANSLATION_KI.get(),
                            AUTO_TRANSLATION_KD.get());
                },
                AUTO_TRANSLATION_KP,
                AUTO_TRANSLATION_KI,
                AUTO_TRANSLATION_KD);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                () ->
                        autoThetaController.setPID(
                                AUTO_THETA_KP.get(), AUTO_THETA_KI.get(), AUTO_THETA_KD.get()),
                AUTO_THETA_KP,
                AUTO_THETA_KI,
                AUTO_THETA_KD);

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

        double[] sampleTimestamps = modules[0].getOdometryTimestamps();
        int sampleCount = sampleTimestamps.length;

        for (int sampleIndex = 0; sampleIndex < sampleCount; sampleIndex++) {
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            for (int i = 0; i < 4; i++) {
                modulePositions[i] = modules[i].getOdometryPositions()[sampleIndex];
            }

            Optional<Rotation2d> gyroAngle = Optional.empty();
            if (gyroInputs.connected) {
                gyroAngle = Optional.of(gyroInputs.yawPosition);
            }

            robotState.addOdometryObservation(
                    new OdometryObservation(
                            Seconds.of(sampleTimestamps[sampleIndex]),
                            modulePositions,
                            gyroAngle,
                            new boolean[] {false, false, false, false}));
        }

        // Update RobotState velocity
        robotState.setRobotRelativeVelocity(getChassisSpeeds());

        Logger.recordOutput(
                "Drive/Speed",
                new Translation2d(
                                        getChassisSpeeds().vxMetersPerSecond,
                                        getChassisSpeeds().vyMetersPerSecond)
                                .getNorm()
                        * -1);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.kSpeedAt12Volts);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /**
     * Runs the drive in a straight line with the specified drive output.
     *
     * @param output Drive output voltage (-12 to 12)
     */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /** Returns a command that follows the supplied Choreo trajectory. */
    public Command followTrajectory(Trajectory<SwerveSample> trajectory) {
        return Commands.sequence(
                        runOnce(
                                () -> {
                                    autoXController.reset();
                                    autoYController.reset();
                                    autoThetaController.reset();
                                    Logger.recordOutput(
                                            "Odometry/Trajectory", trajectory.getPoses());
                                }),
                        createTrajectoryFollower(trajectory))
                .finallyDo(
                        () -> {
                            stop();
                            Logger.recordOutput("Odometry/Trajectory", new Pose2d[] {});
                            Logger.recordOutput("Odometry/TrajectorySetpoint", new Pose2d());
                        })
                .withName("FollowTrajectory_" + trajectory.name());
    }

    /** Follows a single Choreo sample using the drive's autonomous controllers. */
    public void followTrajectorySample(SwerveSample sample) {
        Pose2d currentPose = robotState.getEstimatedPose();
        Pose2d targetPose = sample.getPose();

        ChassisSpeeds targetSpeeds =
                new ChassisSpeeds(
                        sample.vx + autoXController.calculate(currentPose.getX(), sample.x),
                        sample.vy + autoYController.calculate(currentPose.getY(), sample.y),
                        sample.omega
                                + autoThetaController.calculate(
                                        currentPose.getRotation().getRadians(), sample.heading));

        runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, currentPose.getRotation()));
        robotState.setActiveTrajPose(targetPose);
        Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
    }

    /**
     * Returns a command that follows the supplied Choreo trajectory with built-in pause/resume
     * recovery. If the robot is pushed off-path, the trajectory timer freezes and the robot drives
     * back to the paused target before resuming.
     *
     * @param trajectory the trajectory to follow
     * @param eventBindings map of event name to Command scheduled when that event's timestamp is
     *     reached in trajectory-time
     * @return a command that resiliently follows the trajectory
     */
    public ResilientTrajectoryFollower followTrajectoryResilient(
            Trajectory<SwerveSample> trajectory, Map<String, Command> eventBindings) {
        return new ResilientTrajectoryFollower(
                this,
                trajectory,
                autoXController,
                autoYController,
                autoThetaController,
                eventBindings);
    }

    /**
     * Returns a command that follows the supplied Choreo trajectory with built-in pause/resume
     * recovery and no event bindings.
     *
     * @param trajectory the trajectory to follow
     * @return a command that resiliently follows the trajectory
     */
    public ResilientTrajectoryFollower followTrajectoryResilient(
            Trajectory<SwerveSample> trajectory) {
        return followTrajectoryResilient(trajectory, Map.of());
    }

    /** Resets the internal PID state used for trajectory following. */
    public void resetTrajectoryControllers() {
        autoXController.reset();
        autoYController.reset();
        autoThetaController.reset();
    }

    private Command createTrajectoryFollower(Trajectory<SwerveSample> trajectory) {
        final double[] startTime = new double[1];
        return Commands.runEnd(
                        () -> {
                            if (startTime[0] == 0.0) {
                                startTime[0] = Timer.getTimestamp();
                            }
                            double elapsedTime = Timer.getTimestamp() - startTime[0];
                            SwerveSample sample =
                                    trajectory
                                            .sampleAt(elapsedTime, false)
                                            .orElseGet(
                                                    () ->
                                                            trajectory
                                                                    .getFinalSample(false)
                                                                    .orElse(null));
                            if (sample == null) {
                                stop();
                                return;
                            }

                            Pose2d currentPose = robotState.getEstimatedPose();
                            Pose2d targetPose = sample.getPose();

                            ChassisSpeeds targetSpeeds =
                                    new ChassisSpeeds(
                                            sample.vx
                                                    + autoXController.calculate(
                                                            currentPose.getX(), sample.x),
                                            sample.vy
                                                    + autoYController.calculate(
                                                            currentPose.getY(), sample.y),
                                            sample.omega
                                                    + autoThetaController.calculate(
                                                            currentPose.getRotation().getRadians(),
                                                            sample.heading));

                            runVelocity(
                                    ChassisSpeeds.fromFieldRelativeSpeeds(
                                            targetSpeeds, currentPose.getRotation()));
                            robotState.setActiveTrajPose(targetPose);
                            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                        },
                        this::stop,
                        this)
                .until(
                        () ->
                                startTime[0] != 0.0
                                        && Timer.getTimestamp() - startTime[0]
                                                > trajectory.getTotalTime());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules
     * will return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = MODULE_TRANSLATIONS.get(i).getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /**
     * Returns a command to run a quasistatic test in the specified direction.
     *
     * @param direction Direction to run the test (forward or reverse)
     * @return Command that runs the quasistatic test
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction))
                .withName("SysId Quasistatic " + direction.toString());
    }

    /**
     * Returns a command to run a dynamic test in the specified direction.
     *
     * @param direction Direction to run the test (forward or reverse)
     * @return Command that runs the dynamic test
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.dynamic(direction))
                .withName("SysId Dynamic " + direction.toString());
    }

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the modules.
     *
     * @return Array of module positions for all four modules
     */
    protected SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /**
     * Returns the measured chassis speeds of the robot.
     *
     * @return Current chassis speeds in meters per second and radians per second
     */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Returns the position of each module in radians.
     *
     * @return Array of drive positions in radians for all four modules
     */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /**
     * Returns the average velocity of the modules in rotations/sec (Phoenix native units).
     *
     * @return Average drive velocity in rotations per second
     */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /**
     * Returns the maximum linear speed in meters per sec.
     *
     * @return Maximum linear speed in meters per second
     */
    public double getMaxLinearSpeedMetersPerSec() {
        return DriveConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    /**
     * Returns the maximum angular speed in radians per sec.
     *
     * @return Maximum angular speed in radians per second
     */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    /**
     * Returns the acceleration of the gyro in the X direction.
     *
     * @return Acceleration in the X direction
     */
    public LinearAcceleration getAccelerationX() {
        return gyroInputs.xAcceleration;
    }

    /**
     * Returns the acceleration of the gyro in the Y direction.
     *
     * @return Acceleration in the Y
     */
    public LinearAcceleration getAccelerationY() {
        return gyroInputs.yAcceleration;
    }

    /**
     * Returns whether the drivetrain is operating at a significant angle.
     *
     * <p>This checks the current pitch and roll reported by the gyro against the configured maximum
     * allowed angle ({@link DriveConstants#ANGLED_TOLERANCE}). It is used to detect when the robot
     * is on an incline or traversing a bump so that vision-based pose updates can be temporarily
     * ignored while the drivetrain is not level.
     *
     * @return {@code true} if the absolute pitch or roll exceeds the allowed threshold, indicating
     *     the drivetrain is sufficiently angled; {@code false} otherwise.
     */
    public boolean isAngled() {
        if (RobotBase.isSimulation()) {
            return false;
        }

        double pitch = MathUtil.inputModulus(gyroInputs.pitchPosition.getDegrees(), -180.0, 180.0);
        double roll = MathUtil.inputModulus(gyroInputs.rollPosition.getDegrees(), -180.0, 180.0);
        double tolerance = DriveConstants.ANGLED_TOLERANCE.in(Degrees);

        return Math.abs(pitch) > tolerance || Math.abs(roll) > tolerance;
    }

    public Rotation2d getRawGyroAngle() {
        return gyroInputs.yawPosition;
    }
}
