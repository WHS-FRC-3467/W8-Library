package frc.lib.io.lights;

import com.ctre.phoenix6.controls.ControlRequest;
import lombok.Getter;

public class LightsIOSim implements LightsIO {
    @Getter
    private final String name;

    private ControlRequest request;

    public LightsIOSim(String name)
    {
        this.name = name;
    }

    @Override
    public void updateInputs(LightsInputs inputs)
    {
        inputs.connected = true;
        inputs.request = request;
    }

    @Override
    public void setAnimation(ControlRequest request)
    {
        this.request = request;
    }
}
