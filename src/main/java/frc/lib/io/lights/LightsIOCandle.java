package frc.lib.io.lights;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANdle;
import frc.lib.util.CANUpdateThread;
import frc.lib.util.Device;
import lombok.Getter;

public class LightsIOCandle implements LightsIO {
    @Getter
    private final String name;

    private final CANdle candle;

    private final CANUpdateThread updateThread = new CANUpdateThread();

    public LightsIOCandle(String name, Device.CAN id, CANdleConfiguration config)
    {
        this.name = name;

        candle = new CANdle(id.id(), id.bus());

        updateThread.CTRECheckErrorAndRetry(() -> candle.getConfigurator().apply(config));
    }

    @Override
    public void setAnimation(ControlRequest request)
    {
        candle.setControl(request);
    }

}
