package frc.lib.io;

import com.ctre.phoenix6.controls.ControlRequest;

public abstract class LightsIO {

    /**
     * Passes ControlRequest to IO layer
     *
     * @param request {@link ControlRequest}
     */
    public abstract void setAnimation(ControlRequest request);

}
