package frc.lib.io.lights;

import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.controls.ControlRequest;

public interface LightsIO {

    @AutoLog
    abstract class LightsInputs {
        /** Whether the controller is connected. */
        public boolean connected = false;
        /** The current request. */
        public ControlRequest request = null;
    }

    /**
     * Getter for the name of the lights
     * 
     * @return The name of the lights
     */
    public default String getName()
    {
        return "";
    }

    /**
     * Updates the provided {@link LightsInputs} instance with the latest sensor readings. If the
     * sensor is not connected, it populates the fields with default values.
     *
     * @param inputs The structure to populate with updated sensor values.
     */
    public default void updateInputs(LightsInputs inputs)
    {}

    /**
     * Passes ControlRequest to IO layer
     *
     * @param request {@link ControlRequest}
     */
    public default void setAnimation(ControlRequest request)
    {}
}
