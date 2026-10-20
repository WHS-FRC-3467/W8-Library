# `frc.lib` — Library Guide

This package is the reusable library other robot projects copy or depend on. Code here must be
hardware-agnostic at its public surface and must work in all three modes: `REAL`, `SIM`, `REPLAY`.
Read the root [AGENTS.md](../../../../../AGENTS.md) first for build commands and Java style.

## Package map

| Package | Holds | Example to copy |
| --- | --- | --- |
| `io/<thing>/` | IO interface + implementations | `io/beambreak/` (smallest), `io/motor/` (largest) |
| `devices/` | Wrapper owning one IO + its logged inputs | `devices/BeamBreak.java`, `devices/DistanceSensor.java` |
| `mechanisms/` | `Mechanism<T extends MotorIO>` and flywheel / linear / rotary families | `mechanisms/rotary/` |
| `posestimator/` | Pose estimator and swerve odometry | — |
| `autos/`, `commands/` | `Auto` container, `SteppableCommandGroup` | — |
| `util/` | Tunables, logging helpers, geometry, `Device` port records, `CANUpdateThread` | `util/LoggedTunableNumber.java` |

## Adding a new kind of hardware

Create `io/<thing>/` (lowercase, no separators: `distancesensor`, `absoluteencoder`) and follow this
shape. `io/beambreak/` is the minimal reference.

### 1. IO interface — `<Thing>IO.java`

```java
/** Standardized interface for <things> used in FRC. */
public interface ThingIO {

    @AutoLog
    abstract class ThingInputs {
        /** Whether the sensor is connected. */
        public boolean connected = false;

        /** Distance from the sensor to the nearest object. */
        public Distance distance = Meters.zero();
    }

    /**
     * Updates the provided {@link ThingInputs} instance with the latest sensor readings. If the
     * sensor is not connected, it populates the fields with default values.
     *
     * @param inputs The structure to populate with updated sensor values.
     */
    public default void updateInputs(ThingInputs inputs) {}

    /**
     * Runs the thing at the requested output.
     *
     * @param voltage Desired output voltage.
     */
    public default void runVoltage(Voltage voltage) {}
}
```

Rules:

- Inputs class is `@AutoLog abstract class <Thing>Inputs`, nested in the interface, with `public`
  mutable fields, default values, and a `/** */` comment on each field. The annotation processor
  generates `<Thing>InputsAutoLogged`; consumers instantiate that, never the abstract class.
- Inputs fields use WPILib unit types (`Angle`, `Distance`, `AngularVelocity`, `Current`, ...), plus
  `boolean connected` for any device that can drop off the bus. Use `null` only when "no reading" is
  meaningful and expose it as `Optional` from the device wrapper (see `DistanceSensor.getDistance()`).
- **Every method is `default` and a no-op** (or returns a neutral value). Do not make methods
  abstract and do not throw from defaults: replay mode constructs `new ThingIO() {}` and must run.
- Inputs contain only what was *measured*. Commands go in through `run*` / `set*` methods.
- Extend `AutoCloseable` (with a `default void close() {}`) if implementations own hardware handles.

### 2. Implementations — `<Thing>IO<Hardware>.java`, `<Thing>IOSim.java`

- Name by hardware: `MotorIOTalonFX`, `BeamBreakIODIO`, `BeamBreakIOLaserCAN`,
  `DistanceSensorIOCANRange`, `AbsoluteEncoderIOCANCoder`. Simulation: `<Thing>IOSim`; a sim that
  reuses the vendor's own sim API extends the real class: `MotorIOTalonFXSim extends MotorIOTalonFX
  implements MotorIOSim`.
- Constructors take ports as `Device.CAN` / `Device.DIO` / `Device.PWM` records (from
  `frc.lib.util.Device`), never raw ints, plus a `String name` for alerts and logging.
- This is the **only** place vendor classes (`com.ctre`, `au.grapplerobotics`, `org.photonvision`)
  may appear. Nothing outside `io/` should import them.
- Apply vendor configuration asynchronously through `CANUpdateThread`
  (`ctreCheckErrorAndRetry` / `laserCANCheckErrorAndRetry`) so a slow or missing device never blocks
  the robot loop. Log failures with `.exceptionally(...)`.
- CTRE: cache `StatusSignal`s as fields, set update frequencies, call `optimizeBusUtilization`, and
  `BaseStatusSignal.refreshAll(...)` once per `updateInputs`. Reuse preconfigured control request
  objects as `final` fields instead of allocating per loop.
- Report disconnects with a WPILib `Alert` debounced by a `Debouncer` (see `MotorIOTalonFX`).
- Simple sims read operator-controllable values from the dashboard via `LoggedTunableBoolean` /
  `LoggedTunableNumber` (see `BeamBreakIOSim`). Physics sims accept their state from a mechanism
  through setters (see `MotorIOSim.setMechanismPosition`).

### 3. Device wrapper — `devices/<Thing>.java`

A plain class (not a `SubsystemBase`) that a subsystem owns and ticks:

```java
public class Thing {
    private final ThingIO io;
    private final ThingInputsAutoLogged inputs = new ThingInputsAutoLogged();
    private final String name;

    public Thing(String name, ThingIO io) { ... }

    /** Call this method periodically to update sensor readings and log data. */
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    // Getters read only from `inputs`; commands delegate to `io`.
}
```

Getters read from `inputs`, never from `io` or hardware directly — otherwise replay diverges from
the real run.

## Mechanisms

`Mechanism<T extends MotorIO>` wraps a `MotorIO`, logs its inputs, supports tunable PID per
`PIDSlot` (`enableTunablePID`), and optional linear-unit control (`withRadius`). Each family has:

- `<Family>Mechanism<T>` — abstract; family-specific helpers and visualizer updates.
- `<Family>MechanismReal` — binds `T = MotorIO`; usually just a constructor.
- `<Family>MechanismSim` — binds `T = MotorIOSim`; owns a WPILib physics sim
  (`FlywheelSim`, `ElevatorSim`, `SingleJointedArmSim`).
- `<Family>Visualizer` — Mechanism2d / pose output for AdvantageScope.

Sim `periodic()` order matters (see `RotaryMechanismSim`):

1. Feed `inputs.appliedVoltage` into the physics sim and step it by the measured `deltaTime`.
2. Report current draw with `BatterySimCurrentAccumulator.addCurrentLoad(...)` (never set battery
   voltage directly; `Robot` applies the total once per loop).
3. Push mechanism-space position / velocity / acceleration into the `MotorIOSim` setters.
4. Call `super.periodic()` last so logged inputs reflect this step.

All sim state is in **mechanism space** (after gearing). Gear ratios come from the `MotorIO`
(`getRotorToSensorRatio()`, `getSensorToMechanismRatio()`), not from duplicated constants.

Override `setEncoderPosition` in sims to reset the physics state, and `close()` to close every owned
IO (call `super.close()`).

## Utilities worth knowing before writing new ones

- `LoggedTunableNumber` / `LoggedTunableBoolean` / `LoggedTunableProfiledPID` — dashboard-tunable
  values under `Tuning/`; live only when `Constants.tuningMode` is true. React to edits with
  `LoggedTunableNumber.ifChanged(id, action, tunables...)`.
- `PID` record — `P, I, D, A, V, G, S` gains passed to `MotorIO.setPID`.
- `Device` — sealed port records; `Device.CAN` rejects IDs outside 1–63 and duplicate IDs per bus.
- `LoggedTrigger`, `RisingEdge`, `FallingEdge` — logged / edge-detected boolean conditions.
- `GeomUtil`, `FieldUtil`, `PointInPolygon` — geometry and alliance-flip helpers.
- `CANUpdateThread` — async retrying vendor configuration.

Search `util/` before adding a helper; extend an existing one when it fits.

## Library boundaries

- New `frc.lib` code must not import `frc.robot`. A few legacy files read `frc.robot.Constants`
  (tuning mode / current mode) or `FieldConstants`; don't add more. Take what you need as a
  constructor parameter instead.
- No robot-specific numbers (CAN IDs, gear ratios, field poses) in `frc.lib`; those live in the
  robot's `*Constants` / `Ports` classes.
- Public API changes ripple to every team project that uses the library. Prefer adding an overload
  or a new `default` method over changing an existing signature, and update every implementation
  in this repo in the same change.
