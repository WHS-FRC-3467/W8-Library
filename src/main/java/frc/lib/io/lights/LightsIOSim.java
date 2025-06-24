package frc.lib.io.lights;

import java.util.Map;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.controls.ControlRequest;
import lombok.Getter;

public class LightsIOSim implements LightsIO {
    @Getter
    private final String name;

    private Map<String, String> requestInfo;

    public LightsIOSim(String name)
    {
        this.name = name;
    }

    @Override
    public void setAnimation(ControlRequest request)
    {
        this.requestInfo = request.getControlInfo();
        if (requestInfo.containsKey("Slot")) {
            // Logs control request data for each slot
            Logger.recordOutput("LEDs/Slot" + requestInfo.get("Slot"), requestInfo.toString());
        }
    }
}
