package frc.robot.subsystems.GenericRollerSubsystem;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.util.drivers.Phoenix6Util;
import frc.robot.util.drivers.TalonUtil;

/** Generic roller IO implementation for a roller or series of rollers using a Kraken. */
public abstract class GenericRollerSubsystemIOTalonFX implements GenericRollerSubsystemIO {
  
	// Local motor object
	private final TalonFX mMainMotor;

	// Local motor config
	private final TalonFXConfiguration mMainConfig;

	// All the Talon StatusSignals of interest
	private final StatusSignal<Double> mAppliedVoltage;
	private final StatusSignal<Double> mSupplyCurrent;
	private final StatusSignal<Double> mTorqueCurrent;
	private final StatusSignal<Double> mTempCelsius;

  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut mVoltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);

  /*
	 * Constructor
	 */
  public GenericRollerSubsystemIOTalonFX(GenericRollerSubsystemConstants constants) {

    GenericRollerSubsystemConstants mConst = constants;
    
 		// Instantiate the main TalonFX object
		mMainMotor = new TalonFX(mConst.kMainMotorID.getDeviceNumber(), mConst.kMainMotorID.getBus());

		// Get the motor configuration group and configure the main motor
		mMainConfig = mConst.kMotorConfig;
		TalonUtil.applyAndCheckConfiguration(mMainMotor, mMainConfig);

		// Send a neutral command to halt the main motor.
		stop();

		// Assign StatusSignals to our local variables
    mAppliedVoltage = mMainMotor.getMotorVoltage();
    mSupplyCurrent = mMainMotor.getSupplyCurrent();
    mTorqueCurrent = mMainMotor.getTorqueCurrent();
    mTempCelsius = mMainMotor.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, mAppliedVoltage, mSupplyCurrent, mTorqueCurrent, mTempCelsius);

		// Set update frequencies for some basic output signals
		Phoenix6Util.checkErrorAndRetry(() -> mMainMotor.getBridgeOutput().setUpdateFrequency(200, 0.05));
		Phoenix6Util.checkErrorAndRetry(() -> mMainMotor.getFault_Hardware().setUpdateFrequency(4, 0.05));

		// Set update frequencies for the StatusSignals of interest
		Phoenix6Util.checkErrorAndRetry(() -> BaseStatusSignal.setUpdateFrequencyForAll(
			100,
			mAppliedVoltage,
			mSupplyCurrent,
			mTorqueCurrent,
			mTempCelsius
			)
		);
		mMainMotor.optimizeBusUtilization(0, 1.0);
	}

  @Override
  public void updateInputs(GenericRollerIOInputs inputs) {

    // Refresh all signals of interest
    inputs.connected =
        BaseStatusSignal.refreshAll(
                mAppliedVoltage, mSupplyCurrent, mTorqueCurrent, mTempCelsius)
            .isOK();

		// Get voltage, currents, and temperature for the main motor
    inputs.appliedVoltage = mAppliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = mSupplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = mTorqueCurrent.getValueAsDouble();
    inputs.tempCelsius = mTempCelsius.getValueAsDouble();
  }

  @Override
  /** Run roller at specified voltage */
  public void runVolts(double volts) {
    mMainMotor.setControl(mVoltageOut.withOutput(volts));
  }

  @Override
 	/** Stop in Open Loop */
	public void stop() {
		runVolts(0.0);
		mMainMotor.stopMotor();
	}

}
