package frc.lib.io;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.interfaces.LaserCanInterface;
import frc.robot.util.LoggedTunableNumber;

public class LaserCANSim implements LaserCanInterface {

  private LoggedTunableNumber tunableDistance;

  private RangingMode rangingMode;
  private TimingBudget timingBudget;
  private RegionOfInterest regionOfInterest;

  public LaserCANSim(String name) {
    tunableDistance = new LoggedTunableNumber(name + "/Measurement", Double.POSITIVE_INFINITY);
  }

  @Override
  public Measurement getMeasurement() {
    return new Measurement(
        LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT,
        (int) (tunableDistance.get() * 1000),
        0,
        rangingMode == RangingMode.LONG,
        timingBudget.asMilliseconds(),
        regionOfInterest);
  }

  @Override
  public void setRangingMode(RangingMode mode) throws ConfigurationFailedException {
    rangingMode = mode;
  }

  @Override
  public void setTimingBudget(TimingBudget budget) throws ConfigurationFailedException {
    timingBudget = budget;
  }

  @Override
  public void setRegionOfInterest(RegionOfInterest roi) throws ConfigurationFailedException {
    regionOfInterest = roi;
  }
}
