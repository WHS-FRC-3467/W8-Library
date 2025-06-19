package frc.lib.io.beambreak;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.CanDeviceId;
import frc.robot.Robot;
import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.function.Supplier;

public class BeamBreakIOCANrange implements BeamBreakIO {

  private final Debouncer debouncer;
  private final String name;

  private final CANrange CanRange;
  private CANrangeConfiguration configuration;

  private final StatusSignal<Voltage> supplyVoltage;
  private final StatusSignal<Double> ambientSignal;
  private final StatusSignal<Angle> fovCenterX;
  private final StatusSignal<Angle> fovCenterY;
  private final StatusSignal<Angle> fovRangeX;
  private final StatusSignal<Angle> fovRangeY;
  private final StatusSignal<Boolean> isDetected;
  private double lastSignalStrength;
  private final StatusSignal<Double> signalStrength;
  private final double signalStrengthThreshold;

  // Executor for retrying config operations asynchronously
  private BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
  private ThreadPoolExecutor threadPoolExecutor =
      new ThreadPoolExecutor(1, 1, 5, java.util.concurrent.TimeUnit.MILLISECONDS, queue);

  /**
   * Attempts a config action up to 5 times until it succeeds.
   *
   * @param action The status-returning operation to retry.
   */
  public void checkErrorAndRetry(Supplier<StatusCode> action) {
    threadPoolExecutor.submit(
        () -> {
          for (int i = 0; i < 5; i++) {
            StatusCode result = action.get();
            if (result.isOK()) {
              break;
            }
          }
        });
  }

  public BeamBreakIOCANrange(
      CanDeviceId id,
      String name,
      CANrangeConfiguration config,
      Time debounce,
      double signalStrengthThreshold) {

    // Instantiate the CANrange sensor
    debouncer = new Debouncer(debounce.in(Units.Seconds), DebounceType.kBoth);
    this.name = name;
    this.CanRange = new CANrange(id.getDeviceNumber());
    this.signalStrengthThreshold = signalStrengthThreshold;
    // Apply the configuration
    checkErrorAndRetry(() -> CanRange.getConfigurator().apply(config));
    this.configuration = config;

    supplyVoltage = CanRange.getSupplyVoltage();
    ambientSignal = CanRange.getAmbientSignal();
    fovCenterX = CanRange.getRealFOVCenterX();
    fovCenterY = CanRange.getRealFOVCenterY();
    fovRangeX = CanRange.getRealFOVRangeX();
    fovRangeY = CanRange.getRealFOVRangeY();
    isDetected = CanRange.getIsDetected();
    signalStrength = CanRange.getSignalStrength();
    lastSignalStrength = signalStrength.getValue();

    // Set update frequencies for the StatusSignals of interest
    checkErrorAndRetry(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100, supplyVoltage, isDetected, ambientSignal));

    checkErrorAndRetry(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                1, fovCenterX, fovCenterY, fovRangeX, fovRangeY));

    CanRange.optimizeBusUtilization(0, 1.0);
  }

  /**
   * Updates the BeamBreakIO inputs
   *
   * @param inputs The BeamBreakIOInputs object where the updated values will be stored.
   */
  @Override
  public void updateInputs(BeamBreakIOInputs inputs) {
    lastSignalStrength = signalStrength.getValue();
    // Refreshes the signals for detection and other values
    inputs.connected =
        BaseStatusSignal.refreshAll(
                supplyVoltage,
                isDetected,
                fovCenterX,
                fovCenterY,
                fovRangeX,
                fovRangeY,
                ambientSignal,
                signalStrength)
            .isOK();

    // Updates the inputs with the CANrange's values
    inputs.isDetected = isDetected.getValue() && lastSignalStrength > signalStrengthThreshold;
    inputs.supplyVoltage = supplyVoltage.getValue();
    inputs.ambientSignal = ambientSignal.getValue();
    inputs.signalStrength = signalStrength.getValue();
  }

  /**
   * Config constants in this order: centerX, centerY, rangeX, and rangeY
   *
   * @return the Field of View (FOV) configs as a list of Angle objects
   */
  public List<Angle> getFovConfig() {
    return List.of(
        fovCenterX.getValue(), fovCenterY.getValue(), fovRangeX.getValue(), fovRangeY.getValue());
  }

  /**
   * Use this method only if the CANrange is used like a simple beambreak sensor. The beam is
   * considered broken if the last signal strength exceeds the threshold.
   *
   * @return whether the beam is broken based on the last signal strength measurement.
   */
  @Override
  public boolean get() {
    return isDetected.getValue() && lastSignalStrength > signalStrengthThreshold;
  }

  /**
   * Updates the current settings that determine whether the CANrange 'detects' an object. Call this
   * method to achieve the desired performance.
   *
   * @param proximityParamsConfigs The proximity parameters to set, including ProximityThreshold,
   *     ProximityHysteresis, and MinSignalStrengthForValidMeasurement.
   */
  private void setProximityParams(ProximityParamsConfigs proximityParamsConfigs) {
    this.configuration.ProximityParams = proximityParamsConfigs;
    checkErrorAndRetry(() -> CanRange.getConfigurator().apply(this.configuration));
  }

  /**
   * Updates the current settings that determine whether the center and extent of the CANrange's
   * view Call this method to achieve the desired performance.
   *
   * @param fovParamsConfigs The FOV parameters to set, including centerX, centerY, rangeX, and
   *     rangeY.
   */
  private void setFOVParams(FovParamsConfigs fovParamsConfigs) {
    this.configuration.FovParams = fovParamsConfigs;
    checkErrorAndRetry(() -> CanRange.getConfigurator().apply(this.configuration));
  }

  @Override
  public boolean getDebounced() {
    return debouncer.calculate(get());
  }

  @Override
  public boolean getDebouncedIfReal() {
    return Robot.isReal() && getDebounced();
  }

  @Override
  public Command stateWait(boolean state) {
    return Commands.waitUntil(() -> get() == state);
  }

  @Override
  public Command stateWaitWithDebounce(boolean state) {
    return Commands.waitUntil(() -> getDebounced() == state);
  }

  @Override
  public Command stateWaitIfReal(boolean state, double waitSecondsSim) {
    return Commands.either(
        stateWait(state), Commands.waitSeconds(waitSecondsSim), () -> Robot.isReal());
  }

  @Override
  public Command stateWaitWithDebounceIfReal(boolean state, double waitSecondsSim) {
    return Commands.either(
        stateWaitWithDebounce(state), Commands.waitSeconds(waitSecondsSim), () -> Robot.isReal());
  }
}
