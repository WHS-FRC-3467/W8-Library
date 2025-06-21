package frc.lib.io.distancesensor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.Device;
import frc.lib.util.CANUpdateThread;
import lombok.Getter;

/**
 * A distance sensor implementation that uses a CANRange
 */
public class DistanceSensorCANRange implements DistanceSensor {
    @Getter
    private final String name;
    @Getter
    private final Device.CAN id;
    private final CANrange CANRange;

    private final CANUpdateThread updateThread = new CANUpdateThread();

    private final StatusSignal<Distance> distance;
    private final StatusSignal<Double> ambientSignal;

    /**
     * Constructs a {@link DistanceSensorCANRange} object with the specified CAN ID, name, and
     * configuration.
     *
     * @param id The CANDevice identifying the bus and device ID for this sensor.
     * @param name A human-readable name for this sensor instance.
     * @param config The CANrangeConfiguration to apply to the sensor upon initialization.
     */
    public DistanceSensorCANRange(Device.CAN id, String name, CANrangeConfiguration config)
    {
        this.name = name;
        this.id = id;

        CANRange = new CANrange(id.id(), id.bus());

        updateThread.CTRECheckErrorAndRetry(() -> CANRange.getConfigurator().apply(config));

        ambientSignal = CANRange.getAmbientSignal();
        distance = CANRange.getDistance();
    }

    @Override
    public void updateInputs(DistanceSensorInputs inputs)
    {
        inputs.connected = BaseStatusSignal.refreshAll(ambientSignal, distance).isOK();

        if (!inputs.connected) {
            inputs.ambientSignal = 0.0;
            inputs.distance = null;
            return;
        }

        inputs.ambientSignal = ambientSignal.getValue();
        inputs.distance = distance.getValue();
    }

}
