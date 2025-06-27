// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rotarysubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorIOTalonFX;
import frc.robot.Ports;
import frc.robot.Robot;

/** Add your docs here. */
public class RotarySubsystemConstants {
    public static String NAME = "Rotary Subsystem";
    public static final Angle TOLERANCE = Degrees.of(2.0);
    public static final AngularVelocity CRUISE_VELOCITY = Units.RadiansPerSecond.of(2 * Math.PI);
    public static final AngularAcceleration ACCELERATION =
        CRUISE_VELOCITY.div(0.1).per(Units.Second);
    public static final Velocity<AngularAccelerationUnit> JERK = ACCELERATION.per(Second);

    private static final double GEARING = (2.0 / 1.0);
    private static final Angle MIN_ANGLE = Degrees.of(10.0);
    private static final Angle MAX_ANGLE = Degrees.of(80.0);


    public static TalonFXConfiguration getFXConfig()
    {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.StatorCurrentLimit = 80.0;

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE.in(Units.Rotations);

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE.in(Units.Rotations);

        config.Feedback.SensorToMechanismRatio = GEARING;
        return config;
    }

    public static MotorIO getReal()
    {
        return new MotorIOTalonFX(NAME + "Motor", getFXConfig(), Ports.RotarySubsystemMotorMain,
            Ports.RotarySubsystemMotorFollower);
    }

    public static MotorIO getSim()
    {
        return new MotorIO() {}; // TODO: Change to sim motor when implemented
    }

    public static MotorIO getReplay()
    {
        return new MotorIO() {};
    }
}
