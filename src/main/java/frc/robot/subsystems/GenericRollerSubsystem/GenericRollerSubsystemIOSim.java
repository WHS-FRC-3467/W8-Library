package frc.robot.subsystems.GenericRollerSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class GenericRollerSubsystemIOSim implements GenericRollerSubsystemIO {
  private final DCMotorSim sim;
  private double appliedVoltage = 0.0;

  public GenericRollerSubsystemIOSim(DCMotor motorModel, double reduction, double moi) {
    sim = new DCMotorSim(motorModel, reduction, moi);
  }

  @Override
  public void updateInputs(GenericRollerIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      runVolts(0.0);
    }

    sim.update(Constants.loopPeriodSecs);
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void runVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

}
