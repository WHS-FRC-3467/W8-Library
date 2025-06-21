package frc.lib.io.distancesensor;

import frc.lib.util.CANDevice;

/** A default implementation with placeholder methods to allow for log replay */
public class DistanceSensorReplay implements DistanceSensor {

    @Override
    public String getName()
    {
        return "";
    }

    @Override
    public CANDevice getId()
    {
        return new CANDevice();
    }

    @Override
    public void updateInputs(DistanceSensorInputs inputs)
    {}
}
