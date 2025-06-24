package frc.lib.io.lights;

import com.ctre.phoenix6.controls.ControlRequest;

public interface LightsIO {

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
     * Passes ControlRequest to IO layer
     *
     * @param request {@link ControlRequest}
     */
    public default void setAnimation(ControlRequest request)
    {}
}
