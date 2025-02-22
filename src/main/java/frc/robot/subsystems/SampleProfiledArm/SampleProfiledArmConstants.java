package frc.robot.subsystems.SampleProfiledArm;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Ports;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemConstants;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemConstants.simType;

/** Add your docs here. */
public final class SampleProfiledArmConstants {

    public static final GenericMotionProfiledSubsystemConstants kSubSysConstants = new GenericMotionProfiledSubsystemConstants();

    static {

        kSubSysConstants.kName = "SampleProfiledArm";

        kSubSysConstants.kLeaderMotor = Ports.ARM_MAIN;
        kSubSysConstants.kFollowMotor = Ports.ARM_FOLLOWER;
        kSubSysConstants.kFollowerOpposesMain = true;

        // Using TalonFX internal encoder
        
        kSubSysConstants.kCANcoder = null;
		kSubSysConstants.kMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
		kSubSysConstants.kMotorConfig.Feedback.SensorToMechanismRatio = 54.4;
		kSubSysConstants.kMotorConfig.Feedback.RotorToSensorRatio = 1.0;
        
        // Using a remote CANcoder
        /*
        kSubSysConstants.kCANcoder = Ports.ARM_CANCODER;
		kSubSysConstants.kMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
		kSubSysConstants.kMotorConfig.Feedback.SensorToMechanismRatio = 7.04;
		kSubSysConstants.kMotorConfig.Feedback.RotorToSensorRatio = 54.4/7.04;
        kSubSysConstants.kEncoderConfig.MagnetSensor.MagnetOffset = 0.3467;
        kSubSysConstants.kEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        kSubSysConstants.kEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        */

        kSubSysConstants.kMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kSubSysConstants.kMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        kSubSysConstants.kMotorConfig.Voltage.PeakForwardVoltage = 12.0;
        kSubSysConstants.kMotorConfig.Voltage.PeakReverseVoltage = -12.0;

        kSubSysConstants.kMotorConfig.Slot0.kP = 700; // output per unit of error in position (output/rotation)
        kSubSysConstants.kMotorConfig.Slot0.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
        kSubSysConstants.kMotorConfig.Slot0.kD = 100; // output per unit of error derivative in position (output/rps)

        kSubSysConstants.kMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        kSubSysConstants.kMotorConfig.Slot0.kG = 10; // output to overcome gravity (output)
        kSubSysConstants.kMotorConfig.Slot0.kS = 0; // output to overcome static friction (output)
        kSubSysConstants.kMotorConfig.Slot0.kV = 0.19; // output per unit of requested velocity (output/rps)
        kSubSysConstants.kMotorConfig.Slot0.kA = 0; // unused, as there is no target acceleration

        kSubSysConstants.kMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 500;
        kSubSysConstants.kMotorConfig.MotionMagic.MotionMagicAcceleration = 50;
        kSubSysConstants.kMotorConfig.MotionMagic.MotionMagicJerk = 0;
        
        kSubSysConstants.kMotorConfig.CurrentLimits.SupplyCurrentLimit = 20;
        kSubSysConstants.kMotorConfig.CurrentLimits.SupplyCurrentThreshold = 40;
        kSubSysConstants.kMotorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
        kSubSysConstants.kMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        kSubSysConstants.kMotorConfig.CurrentLimits.StatorCurrentLimit = 70;
        kSubSysConstants.kMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // Simulation Type
        kSubSysConstants.SimType = simType.ARM;

        // Motor simulation
        kSubSysConstants.kMotorSimConfig.simMotorModelSupplier = ()-> DCMotor.getKrakenX60Foc(2);
        
        // Arm Simulation
        kSubSysConstants.kArmSimConfig.kArmMass = 8.0; // Kilograms
        kSubSysConstants.kArmSimConfig.kArmLength = Units.inchesToMeters(30);
        kSubSysConstants.kArmSimConfig.kDefaultArmSetpointDegrees = 75.0;
        kSubSysConstants.kArmSimConfig.kMinAngleDegrees = -75.0;
        kSubSysConstants.kArmSimConfig.kMaxAngleDegrees = 255.0;
        kSubSysConstants.kArmSimConfig.kArmReduction = 54.4; // RotorToSensorRatio * SensorToMechanismRatio
        kSubSysConstants.kArmSimConfig.kSensorReduction = 7.04; // SensorToMechanismRatio
         
    }
}

 
