package frc.lib.io;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.ControlRequest;

public class LightsIOSim extends LightsIO {
    private Map<String, String> requestInfo;

    public LightsIOSim() {
        super();

    }

    @Override
    public void setAnimation(ControlRequest request) {
        this.requestInfo = request.getControlInfo();
        if (requestInfo.containsKey("Slot")) {
            Logger.recordOutput("LEDs/Slot" + requestInfo.get("Slot"), requestInfo.toString()); // Logs control request
                                                                                                // data for each slot
        }
    }

}
