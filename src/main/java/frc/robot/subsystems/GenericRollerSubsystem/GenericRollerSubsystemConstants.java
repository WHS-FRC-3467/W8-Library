package frc.robot.subsystems.GenericRollerSubsystem;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.drivers.CanDeviceId;

/**
 * Wrapper class for TalonFX config params
 * (Recommend initializing in a static block!)
 */
public class GenericRollerSubsystemConstants {

    public String kName = "ERROR_ASSIGN_A_NAME";

	public List<CanDeviceId> kMotorIDs;

	public TalonFXConfiguration kMotorConfig = new TalonFXConfiguration();

	public Supplier<DCMotor> simMotorModelSupplier = ()-> DCMotor.getKrakenX60Foc(1);
 	public final double simReduction = (18.0 / 12.0);
  	public final double simMOI = 0.001;

}
