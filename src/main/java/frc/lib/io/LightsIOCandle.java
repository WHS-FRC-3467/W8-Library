package frc.lib.io;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANdle;

public class LightsIOCandle extends LightsIO {
    private final CANdle candle;

    public LightsIOCandle(LightsIOCandleConfiguration config) {
        super();
        candle = new CANdle(config.id, config.bus);
        candle.getConfigurator().apply(config.candleConfig);
    }

    public static class LightsIOCandleConfiguration {
        public int id;
        public String bus;
        public CANdleConfiguration ledConfig = new CANdleConfiguration();
        public CANdleFeaturesConfigs candleConfig = new CANdleFeaturesConfigs();
    }

    @Override
    public void setAnimation(ControlRequest request) {
        candle.setControl(request);
    }

}
