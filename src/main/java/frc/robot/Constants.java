// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;
//Test

/** Add your docs here. */
public class Constants {

        public static final Mode currentMode = Mode.REAL;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }


    public static final class ExampleSimpleSubsystemConstants {
        public static final int ID_Motor = 0;

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0;

            m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
            m_configuration.CurrentLimits.StatorCurrentLimit = 70;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;

            return m_configuration;
        }
    }

    public static final class ExampleComplexSubsystemConstants {
        public static final int ID_Motor = 1;
        public static final double upperLimit = Units.degreesToRadians(180);
        public static final double lowerLimit = Units.degreesToRadians(0);
        public static final double tolerance = Units.degreesToRadians(1);

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0;

            m_configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            m_configuration.Feedback.SensorToMechanismRatio = 1;

            m_configuration.Slot0.kP = 1; // output per unit of error in position (output/rotation)
            m_configuration.Slot0.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
            m_configuration.Slot0.kD = 0; // output per unit of error derivative in position (output/rps)

            m_configuration.Slot1.kG = 0; // output to overcome gravity (output)
            m_configuration.Slot1.kS = 0; // output to overcome static friction (output)
            m_configuration.Slot1.kV = 0; // output per unit of requested velocity (output/rps)
            m_configuration.Slot1.kA = 0; // unused, as there is no target acceleration
            m_configuration.Slot1.kP = 1; // output per unit of error in position (output/rotation)
            m_configuration.Slot1.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
            m_configuration.Slot1.kD = 0; // output per unit of error derivative in position (output/rps)

            m_configuration.MotionMagic.MotionMagicCruiseVelocity = 10;
            m_configuration.MotionMagic.MotionMagicAcceleration = 10;
            m_configuration.MotionMagic.MotionMagicJerk = 10;

            m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
            m_configuration.CurrentLimits.StatorCurrentLimit = 70;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;

            return m_configuration;
        }
    }

    public static final class ArmConstants { 
        public static final int ID_ArmLeader = 25;
        public static final int ID_ArmFollower = 26;  
        public static final int ID_ArmAbsEncoder = 0;

        public static final double upperLimit = Units.degreesToRadians(130);
        public static final double lowerLimit = Units.degreesToRadians(-20);
        public static final double tolerance = Units.degreesToRadians(.5);
        public static final double maxVelocity = Math.PI;
        public static final double maxAcceleration = Math.PI / 2;

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            m_configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            m_configuration.MotorOutput.DutyCycleNeutralDeadband = .005;
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0; 

            m_configuration.CurrentLimits.SupplyCurrentLimit = 30; 
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 50;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1; 
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true; 
            m_configuration.CurrentLimits.StatorCurrentLimit = 60;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true; 

            return m_configuration;
        }
    }

    public static final class DriveConstants {
        public static final double headingAngleTolerance = 2.0;
        public static final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
        public static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.CurrentLimits.SupplyCurrentLimit = 30; 
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 90;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.01; 
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true; 
            m_configuration.CurrentLimits.StatorCurrentLimit = 80;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true; 

            return m_configuration;
        }
    }

    public static final class IntakeConstants { 
        public static final int ID_IntakeMotor = 19;
        public static final int ID_IntakeFollower = 21;

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0; 

            m_configuration.CurrentLimits.SupplyCurrentLimit = 20; 
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1; 
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true; 
            m_configuration.CurrentLimits.StatorCurrentLimit = 70;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true; 

            return m_configuration;
        }
    }

    public static final class ShooterConstants { 
        public static final int ID_ShooterLeft = 15;
        public static final int ID_ShooterRight = 17;

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            m_configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0; 

            m_configuration.CurrentLimits.SupplyCurrentLimit = 60; 
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 80;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1; 
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true; 
            m_configuration.CurrentLimits.StatorCurrentLimit = 70;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true; 

            m_configuration.Slot0.kP = .03;
            m_configuration.Slot0.kI = 0;
            m_configuration.Slot0.kD = 0;
            m_configuration.Slot0.kV = .125;

            return m_configuration;
        }

        public static TalonFXConfiguration motor2Config() {
            TalonFXConfiguration m_configuration = motorConfig();
            m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            return m_configuration;
        }
    }

    public static final class StageConstants { 
        // Stage CAN IDs
        public static final int ID_StageMotor = 23;
        public static final int ID_StageBeamBreak = 1;

/*         public static TalonFXConfiguration StageConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0; 

            m_configuration.CurrentLimits.SupplyCurrentLimit = 20; 
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1; 
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true; 
            m_configuration.CurrentLimits.StatorCurrentLimit = 70;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true; 

            return m_configuration;
        } */
    }

    public static final class ControllerConstants {

        public static final double leftTriggerMin = 0.3;
        public static final double leftTriggerMid = 0.6;

    }

    public static class FieldConstants {

        public static final Pose2d BLUE_SPEAKER = new Pose2d(Units.inchesToMeters(-1.5 + 12), Units.inchesToMeters(218.42), new Rotation2d(0));
        public static final Pose2d RED_SPEAKER = new Pose2d(Units.inchesToMeters(652.73 - 12), Units.inchesToMeters(218.42), new Rotation2d(Math.PI));
        public static final Pose2d BLUE_FEED = new Pose2d(1.25, 6.62, new Rotation2d(0));
        public static final Pose2d RED_FEED = new Pose2d(15.250, 6.62, new Rotation2d(0));
        public static final Pose2d BLUE_AMP = new Pose2d(Units.inchesToMeters(72.5),Units.inchesToMeters(323.00),new Rotation2d(Math.PI/2));
        public static final Pose2d RED_AMP = new Pose2d(Units.inchesToMeters(578.77),Units.inchesToMeters(323.00),new Rotation2d(-Math.PI/2));
        public static final double BLUE_AUTO_PENALTY_LINE = 9; // X distance from origin to center of the robot almost fully crossing the midline
        public static final double RED_AUTO_PENALTY_LINE = 7.4; // X distance from origin to center of the robot almost fully crossing the midline

        public static final Rotation2d ampAngle = new Rotation2d(Math.PI / 2);
    }

}

