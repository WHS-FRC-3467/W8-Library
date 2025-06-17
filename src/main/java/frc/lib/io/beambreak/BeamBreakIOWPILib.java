package frc.lib.io.beambreak;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.hal.SimDevice;

import frc.robot.Constants;

/**
 * The BeamBreakIOWPILib class represents a BeamBreak interface for WPILib's DigitalInput class. 
 * It integrates with beam break sensors and limit switches and provides methods for getting data from the sensor.
 */
public class BeamBreakIOWPILib implements BeamBreakIO {

    private final String name;

    private final DigitalInput m_BeamBreak;

    /**
     * Constructor for BeamBreakIOTalonFX that initializes the BeamBreak, its status signals, and applies
     * configurations for motion control.
     *
     * @param id The DIO port number of the sensor.
     */
    public BeamBreakIOWPILib(String name, int id)
    {
        this.name = name;

        m_BeamBreak = new DigitalInput(id);
        if (Constants.currentMode == Constants.Mode.SIM) {
            setSimDevice();
        }
    }

    /**
     * Updates the BeamBreak input for tracking the sensor's state.
     *
     * @param inputs The BeamBreakIOInputs object where the updated values will be stored.
     */
    @Override
    public void updateInputs(BeamBreakIOInputs inputs)
    {
        inputs.isDetected = m_BeamBreak.get();
    }

    /**
     * Get the state of the digital input
     */
    @Override
    public boolean get() 
    {
        return m_BeamBreak.get();
    }

    /**
     * Get the channel of the digital input.
     *
     * @return The GPIO channel number that this object represents.
     */
    public int getChannel() 
    {
        return m_BeamBreak.getChannel();
    }

    /**
     * Get the analog trigger type.
     *
     * @return false
     */
    public int getAnalogTriggerTypeForRouting() 
    {
        return m_BeamBreak.getAnalogTriggerTypeForRouting();
    }

    /**
     * Close the digital input
     */
    public void close()
    {
        m_BeamBreak.close();
    }

    /**
     * Indicates that the input is used by a simulated device.
     */
    public void setSimDevice()
    {
        m_BeamBreak.setSimDevice(SimDevice.create(name));
    }

}