package frc.lib.io.distancesensor;

import frc.lib.util.Device;

/** A default implementation with placeholder methods to allow for log replay */
public class DistanceSensorReplay implements DistanceSensor {

    @Override
    public String getName()
    {
        return "";
    }

    @Override
    public Device.CAN getId()
    {
        return new Device.CAN(0, "");
    }

    @Override
    public void updateInputs(DistanceSensorInputs inputs)
    {}
}
