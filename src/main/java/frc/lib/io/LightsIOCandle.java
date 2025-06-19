package frc.lib.io;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.CANdle;

public class LightsIOCandle extends LightsIO {
	private final CANdle candle;

	public LightsIOCandle(LightsIOCandleConfiguration config) {
		super(config.segments);
		candle = new CANdle(config.id, config.bus);
		candle.getConfigurator().apply(config.configuration);
	}

	public static class LightsIOCandleConfiguration {
		public int id;
		public String bus;
		public LEDSegment[] segments;
		public CANdleConfiguration configuration = new CANdleConfiguration();
	}
}
