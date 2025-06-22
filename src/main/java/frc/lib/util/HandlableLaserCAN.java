package frc.lib.util;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

public class HandlableLaserCAN implements AutoCloseable {
    public enum ConfigurationStatus {
        SUCCESS,
        FAILURE;
    }

    private final LaserCan laserCAN;

    public HandlableLaserCAN(int can_id)
    {
        laserCAN = new LaserCan(can_id);
    }

    public Measurement getMeasurement()
    {
        return laserCAN.getMeasurement();
    }

    public ConfigurationStatus setRangingMode(RangingMode mode)
    {
        try {
            laserCAN.setRangingMode(mode);
            return ConfigurationStatus.SUCCESS;
        } catch (ConfigurationFailedException exception) {
            return ConfigurationStatus.FAILURE;
        }
    }

    public ConfigurationStatus setTimingBudget(TimingBudget budget)
    {
        try {
            laserCAN.setTimingBudget(budget);
            return ConfigurationStatus.SUCCESS;
        } catch (ConfigurationFailedException exception) {
            return ConfigurationStatus.FAILURE;
        }
    }

    public ConfigurationStatus setRegionOfInterest(RegionOfInterest roi)
    {
        try {
            laserCAN.setRegionOfInterest(roi);
            return ConfigurationStatus.SUCCESS;
        } catch (ConfigurationFailedException exception) {
            return ConfigurationStatus.FAILURE;
        }
    }

    @Override
    public void close() throws Exception
    {
        laserCAN.close();
    }
}
