// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

/** Add your docs here. */
public class RobotState {
    private static RobotState instance;

    @Getter
    @Setter
    private Pose2d robotPose = new Pose2d();

    @Getter
    @Setter
    private ChassisSpeeds robotSpeeds = new ChassisSpeeds();

    @RequiredArgsConstructor
    @Getter
    public enum TARGET {
        NONE(null,null),
        SPEAKER(Constants.FieldConstants.BLUE_SPEAKER,Constants.FieldConstants.RED_SPEAKER),
        AMP(Constants.FieldConstants.BLUE_AMP,Constants.FieldConstants.RED_AMP),
        FEED(Constants.FieldConstants.BLUE_FEED,Constants.FieldConstants.RED_FEED);

        private final Pose2d blueTargetPose;
        private final Pose2d redTargetPose;

    }

    @Getter
    @Setter
    private TARGET target = TARGET.NONE;

    private double deltaT = .15; 


    public static RobotState getInstance() {
        if (instance == null)
            instance = new RobotState();
        return instance;
    }

    private Translation2d getFuturePose() {
        // If magnitude of velocity is low enough, use current pose
        if (Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond) < .25) {
            return robotPose.getTranslation();
        } else {
            // Add translation based on current speed and time in the future deltaT
            return robotPose.getTranslation().plus(new Translation2d(deltaT * robotSpeeds.vxMetersPerSecond, deltaT * robotSpeeds.vyMetersPerSecond));
        }
        
    }

    public Rotation2d getAngleOfTarget() {
        // Return the angle to allign to target
        return (DriverStation.getAlliance().get() == Alliance.Blue) ? target.blueTargetPose.getRotation() : target.redTargetPose.getRotation();
    }

    // TODO: need to invert
    public Rotation2d getAngleToTarget() {
        return getFuturePose()
                .minus((DriverStation.getAlliance().get() == Alliance.Blue) ? target.blueTargetPose.getTranslation() : target.redTargetPose.getTranslation())
                .getAngle().unaryMinus(); // TODO: Test if unaryMinus fixed it
    }

    private double getDistanceToTarget() {
        return getFuturePose().getDistance(
                (DriverStation.getAlliance().get() == Alliance.Blue) ? target.blueTargetPose.getTranslation() : target.redTargetPose.getTranslation());
    }

    private static final InterpolatingDoubleTreeMap speakerArmAngleMap = new InterpolatingDoubleTreeMap();
    static {
        speakerArmAngleMap.put(1.5, 12.71);
        speakerArmAngleMap.put(2.0, 21.00);
        speakerArmAngleMap.put(2.5, 24.89);
        speakerArmAngleMap.put(3.0, 29.00);
        speakerArmAngleMap.put(3.5, 31.20);
        speakerArmAngleMap.put(4.0, 32.50);
        speakerArmAngleMap.put(4.5, 34.00);
        speakerArmAngleMap.put(5.0, 35.00);
    }

    private static final InterpolatingDoubleTreeMap feedArmAngleMap = new InterpolatingDoubleTreeMap();
    static {
        feedArmAngleMap.put(5.0, 0.0);
        feedArmAngleMap.put(6.0, -10.0);
        feedArmAngleMap.put(7.0, -19.0);
    }

    public double getShotAngle() {
        switch (target) {
            case SPEAKER:
                return speakerArmAngleMap.get(getDistanceToTarget());
            case FEED:
                return feedArmAngleMap.get(getDistanceToTarget());
            default:
                return 0.0;
        }
    }

    public Command setTargetCommand(TARGET target) {
        return Commands.startEnd(() -> setTarget(target), () -> setTarget(TARGET.NONE))
                .withName("Set Robot Target: " + target.toString());
    }

}
