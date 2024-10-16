package frc.robot.subsystems.GenericRollerSubsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.util.drivers.CanDeviceId;

/**
 * Wrapper class for TalonFX config params
 * (Recommend initializing in a static block!)
 */
public class GenericRollerSubsystemConstants {

    public String kName = "ERROR_ASSIGN_A_NAME";

	public CanDeviceId kMainMotorID = null;

	public TalonFXConfiguration kMotorConfig = new TalonFXConfiguration();

}
