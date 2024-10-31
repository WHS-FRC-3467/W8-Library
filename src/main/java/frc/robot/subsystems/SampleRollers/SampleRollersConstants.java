package frc.robot.subsystems.SampleRollers;

import java.util.List;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Ports;
import frc.robot.subsystems.GenericRollerSubsystem.GenericRollerSubsystemConstants;

/** Add your docs here. */
public final class SampleRollersConstants {

    public static final GenericRollerSubsystemConstants kIntakeConstants = new GenericRollerSubsystemConstants();

    static {

        kIntakeConstants.kName = "SampleRollers";

//        kIntakeConstants.kMotorIDs = List.of(Ports.SAMPLE_ROLLER);
        kIntakeConstants.kMotorIDs = List.of(Ports.TWO_ROLLER_1, Ports.TWO_ROLLER_2);

        kIntakeConstants.kMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kIntakeConstants.kMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        kIntakeConstants.kMotorConfig.Voltage.PeakForwardVoltage = 12.0;
        kIntakeConstants.kMotorConfig.Voltage.PeakReverseVoltage = -12.0;

        kIntakeConstants.kMotorConfig.CurrentLimits.SupplyCurrentLimit = 20;
        kIntakeConstants.kMotorConfig.CurrentLimits.SupplyCurrentThreshold = 40;
        kIntakeConstants.kMotorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
        kIntakeConstants.kMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        kIntakeConstants.kMotorConfig.CurrentLimits.StatorCurrentLimit = 70;
        kIntakeConstants.kMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    }
}

 
