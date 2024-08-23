package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotState;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem {

    @RequiredArgsConstructor
    @Getter
    public enum State {
        TELEOP,
        HEADING,
        CARDINAL;

    }

    @Setter
    private State state = State.TELEOP;

    public State getDrivetrainState() {
        return state;
    }

    private RobotState.TARGET target = RobotState.TARGET.NONE;

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private double controllerX = 0.0;
    private double controllerY = 0.0;
    private double controllerOmega = 0.0;

    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.MaxSpeed * 0.1).withRotationalDeadband(DriveConstants.MaxAngularRate * 0.1) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(DriveConstants.MaxSpeed * 0.1).withRotationalDeadband(DriveConstants.MaxAngularRate * 0.1) 
            .withDriveRequestType(DriveRequestType.Velocity);

    //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        setHeadingPID();
        setSwerveDriveCustomCurrentLimits();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
                new HolonomicPathFollowerConfig(new PIDConstants(5, 0, 0.08),
                        new PIDConstants(5, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void simulationPeriodic() {
        /* Assume 20ms update rate, get battery voltage from WPILib */
        updateSimState(0.02, RobotController.getBatteryVoltage());
    }

    @Override
    public void periodic() {

        RobotState.getInstance().setRobotPose(getState().Pose); //Tell RobotState current pose
        RobotState.getInstance().setRobotSpeeds(getCurrentRobotChassisSpeeds()); //Tell RobotState current speeds
        
        target = RobotState.getInstance().getTarget();

        switch (target) {
            case NONE:
                setState(State.TELEOP);
                break;
            case SPEAKER:
                setState(State.HEADING);
                break;
            case FEED:
                setState(State.HEADING);
                break;
            case AMP:
                setState(State.CARDINAL);
                break;
            default:
                setState(State.TELEOP);
                break;
        }

         switch (state) {
            case TELEOP -> {
                this.setControl(fieldCentric
                .withVelocityX(controllerX * DriveConstants.MaxSpeed)
                .withVelocityY(controllerY * DriveConstants.MaxSpeed)
                .withRotationalRate(controllerOmega * DriveConstants.MaxAngularRate));
            }
            case HEADING -> {
                this.setControl(fieldCentricFacingAngle
                .withVelocityX(controllerX * DriveConstants.MaxSpeed)
                .withVelocityY(controllerY * DriveConstants.MaxSpeed)
                .withTargetDirection(RobotState.getInstance().getAngleToTarget()));
            }
            case CARDINAL -> {
                this.setControl(fieldCentricFacingAngle
                .withVelocityX(controllerX * DriveConstants.MaxSpeed)
                .withVelocityY(controllerY * DriveConstants.MaxSpeed)
                .withTargetDirection(RobotState.getInstance().getAngleOfTarget()));
            }
            default -> {}
        } 

    }

    private void setHeadingPID() {
        if (Robot.isReal()) {
            fieldCentricFacingAngle.HeadingController.setPID(25, 10, 2);
        } else {
            fieldCentricFacingAngle.HeadingController.setPID(5, 0, 0);
        }
        fieldCentricFacingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        fieldCentricFacingAngle.HeadingController.setTolerance(Units.degreesToRadians(.5)); //TODO: confirm this tolerance
    }

    public void setControllerInput(double controllerX, double controllerY, double controllerOmega) {
        this.controllerX = controllerX;
        this.controllerY = controllerY;
        this.controllerOmega = controllerOmega;
    }

    public boolean atGoal() {
        if (state == State.TELEOP) {
            return true;
        } else {
            return fieldCentricFacingAngle.HeadingController.atSetpoint();
        }

    }

    public Command setStateCommand(State state) {
        return startEnd(() -> setState(state), () -> setState(State.TELEOP));
    }

    public void setSwerveDriveCustomCurrentLimits() {
        // Create a current configuration to use for the drive motor of each swerve
        // module.
        var customCurrentLimitConfigs = new CurrentLimitsConfigs();

        // Iterate through each module.
        for (var module : Modules) {
            // Get the Configurator for the current drive motor.
            var currentConfigurator = module.getDriveMotor().getConfigurator();

            // Refresh the current configuration, since the stator current limit has already
            // been set.
            currentConfigurator.refresh(customCurrentLimitConfigs);

            // Set all of the parameters related to the supply current. The values should
            // come from Constants.
            customCurrentLimitConfigs.SupplyCurrentLimit = 30;
            customCurrentLimitConfigs.SupplyCurrentThreshold = 90;
            customCurrentLimitConfigs.SupplyTimeThreshold = .01;
            customCurrentLimitConfigs.SupplyCurrentLimitEnable = true;

            customCurrentLimitConfigs.StatorCurrentLimit = 80;
            customCurrentLimitConfigs.StatorCurrentLimitEnable = true;

            // Apply the new current limit configuration.
            currentConfigurator.apply(customCurrentLimitConfigs);
        }
    }
}
