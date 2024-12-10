package frc.robot.subsystems.GenericRollerSubsystem;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.drivers.TalonUtil;
import frc.robot.util.sim.PhysicsSim;

/**
 * Generic roller IO implementation for a roller or
 * series of rollers using a Kraken.
 */
public abstract class GenericRollerSubsystemIOSim implements GenericRollerSubsystemIO {

  private boolean mInitialized = false;

  private final PhysicsSim mSim;
  private final GenericRollerSimMechanisms mMechanisms;

  // List of commanded voltages (Sim needs to hold this)
  private List<Double> mAppVoltage = new ArrayList<Double>();

  // Local motor object(s)
  private List<TalonFX> mMotors = new ArrayList<TalonFX>();
  private int mNumMotors = 0;

  // Local motor config
  private TalonFXConfiguration mConfig;

  // All the Talon StatusSignals of interest
  private final List<StatusSignal<Double>> mPositionRot = new ArrayList<StatusSignal<Double>>();
  private final List<StatusSignal<Double>> mVelocityRps = new ArrayList<StatusSignal<Double>>();
  private final List<StatusSignal<Double>> mAppliedVoltage = new ArrayList<StatusSignal<Double>>();
  private final List<StatusSignal<Double>> mSupplyCurrent = new ArrayList<StatusSignal<Double>>();
  private final List<StatusSignal<Double>> mTorqueCurrent = new ArrayList<StatusSignal<Double>>();
  private final List<StatusSignal<Double>> mTempCelsius = new ArrayList<StatusSignal<Double>>();

  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut mVoltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);

  /*
   * Constructor
   */
  public GenericRollerSubsystemIOSim(GenericRollerSubsystemConstants constants) {

    GenericRollerSubsystemConstants mConst = constants;
    // Number of motors is based on how many IDs are specified in the provided
    // Constants
    mNumMotors = mConst.kMotorIDs.size();

    // Create the Physics Sim and the Mechanism display
    mSim = PhysicsSim.getInstance();
    mMechanisms = new GenericRollerSimMechanisms(mConst.kName, mNumMotors);

    // Instantiate each roller
    for (int i = 0; i < mNumMotors; i++) {

      // Instantiate the TalonFX object for this roller
      TalonFX mMotor = new TalonFX(mConst.kMotorIDs.get(i).getDeviceNumber(), mConst.kMotorIDs.get(i).getBus());
      mMotors.add(mMotor);
      mAppVoltage.add(0.0);

      // Get the motor configuration group and configure the motor
      mConfig = mConst.kMotorConfig;
      TalonUtil.applyAndCheckConfiguration(mMotor, mConfig);

      // Assign StatusSignals to our local variables
      mPositionRot.add(mMotor.getPosition());
      mVelocityRps.add(mMotor.getVelocity());
      mAppliedVoltage.add(mMotor.getMotorVoltage());
      mSupplyCurrent.add(mMotor.getSupplyCurrent());
      mTorqueCurrent.add(mMotor.getTorqueCurrent());
      mTempCelsius.add(mMotor.getDeviceTemp());

      // Add the motor object to the Physics Sim
      mSim.addTalonFX(mMotor, new DCMotorSim(constants.simMotorModelSupplier.get(),
          constants.simReduction, constants.simMOI));
    }

    // Send a neutral command to halt all motors.
    stop();

  }

  @Override
  public void updateInputs(GenericRollerIOInputs inputs) {

    if (!mInitialized) {
      inputs.connected = new boolean[mNumMotors];
      inputs.positionRot = new double[mNumMotors];
      inputs.velocityRps = new double[mNumMotors];
      inputs.appliedVoltage = new double[mNumMotors];
      inputs.supplyCurrentAmps = new double[mNumMotors];
      inputs.torqueCurrentAmps = new double[mNumMotors];
      inputs.tempCelsius = new double[mNumMotors];

      mInitialized = true;
    }

    // Run the Sim
    mSim.run();

    for (int i = 0; i < mNumMotors; i++) {

      if (DriverStation.isDisabled()) {
        runVolts(i, 0.0);
      }

      TalonFX mFX = mMotors.get(i);
      inputs.connected[i] = true;

 	  // Get velocity, voltage, currents, and temperature for the motor
      inputs.positionRot[i] = mFX.getPosition().getValue();
      inputs.velocityRps[i] = mFX.getVelocity().getValue();
      inputs.appliedVoltage[i] = mAppVoltage.get(i);
      inputs.supplyCurrentAmps[i] = mFX.getSimState().getSupplyCurrent();
      inputs.torqueCurrentAmps[i] = mFX.getSimState().getTorqueCurrent();
      inputs.tempCelsius[i] = mFX.getDeviceTemp().getValue();

      mMechanisms.update(i, inputs.positionRot[i]);
    }

  }

  @Override
  public void runVolts(int index, double volts) {
    mAppVoltage.set(index, MathUtil.clamp(volts, -12.0, 12.0));
    mMotors.get(index).setControl(mVoltageOut.withOutput(mAppVoltage.get(index)));
  }

  @Override
  public void stop() {
    for (int i = 0; i < mNumMotors; i++) {
      runVolts(i, 0.0);
      mMotors.get(i).stopMotor();
    }
  }

}
