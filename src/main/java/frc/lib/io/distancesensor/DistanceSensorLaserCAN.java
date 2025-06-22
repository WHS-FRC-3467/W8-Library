package frc.lib.io.distancesensor;

import static edu.wpi.first.units.Units.Millimeters;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.util.Device;
import frc.lib.util.CANUpdateThread;
import frc.lib.util.HandlableLaserCAN;
import lombok.Getter;

/**
 * A distance sensor implementation that uses a LaserCAN
 */
public class DistanceSensorLaserCAN implements DistanceSensor {
    @Getter
    private final String name;
    private final HandlableLaserCAN laserCAN;

    private final CANUpdateThread updateThread = new CANUpdateThread();

    private final Alert laserCANOnWrongBusAlert;
    private final Alert disconnectedAlert;

    /**
     * Constructs a new {@link DistanceSensorLaserCAN} with specified parameters and configuration.
     *
     * @param id The CAN device ID and bus to which the sensor is connected.
     * @param name A human-readable name for the sensor instance.
     * @param rangingMode The ranging mode to configure on the sensor.
     * @param regionOfInterest The region of interest setting for the sensor.
     * @param timingBudget The timing budget setting that controls measurement speed/accuracy.
     */
    public DistanceSensorLaserCAN(Device.CAN id, String name, RangingMode rangingMode,
        RegionOfInterest regionOfInterest, TimingBudget timingBudget)
    {
        this.name = name;

        laserCANOnWrongBusAlert =
            new Alert("LaserCAN " + name + " must be wired to the RIO's CAN bus", AlertType.kError);
        disconnectedAlert = new Alert("LaserCAN " + name + " is not connected", AlertType.kError);

        if (id.bus() != "rio") {
            laserCANOnWrongBusAlert.set(true);
        }

        laserCAN = new HandlableLaserCAN(id.id());

        updateThread.LaserCANCheckErrorAndRetry(() -> laserCAN.setRangingMode(rangingMode));
        updateThread
            .LaserCANCheckErrorAndRetry(() -> laserCAN.setRegionOfInterest(regionOfInterest));
        updateThread.LaserCANCheckErrorAndRetry(() -> laserCAN.setTimingBudget(timingBudget));
    }

    @Override
    public void updateInputs(DistanceSensorInputs inputs)
    {
        Measurement measure = laserCAN.getMeasurement();

        if (measure == null) {
            disconnectedAlert.set(true);

            inputs.connected = false;
            inputs.ambientSignal = 0.0;
            inputs.distance = null;
            return;
        }

        disconnectedAlert.set(false);

        inputs.connected = true;
        inputs.ambientSignal = measure.ambient;

        if (measure.status != LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT) {
            inputs.distance = null;
            return;
        }

        inputs.distance = Millimeters.of(measure.distance_mm);
    }
}
