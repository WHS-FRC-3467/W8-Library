# Tests Guide

JUnit 5 tests run on the desktop against the WPILib HAL simulation; no robot hardware is needed.
Run with `./gradlew test`. Read the root [AGENTS.md](../../AGENTS.md) first.

## Layout and naming

- `src/test/java/` mirrors `src/main/java/`: a test for `frc.lib.util.LoggedTrigger` lives in
  `src/test/java/frc/lib/util/LoggedTriggerTest.java`.
- Class `<ClassUnderTest>Test`, package-private (`class DriveTest`), test methods package-private
  `void testSomething()` annotated `@Test`.

## Writing a test

```java
class IntakeTest {
    static final double DELTA = 1e-2; // acceptable deviation for floating-point checks

    Intake intake;

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0)); // initialize the HAL, crash if failed

        intake = new Intake(new MotorIOTalonFXSim(...));

        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        Timer.delay(0.100); // let simulated devices start up and enable
    }

    @Test
    void testRunsForward() {
        TestUtil.runTest(intake.intake(), 0.5, intake);
        assertEquals(10.0, intake.getVelocity().in(RotationsPerSecond), DELTA);
    }
}
```

- Initialize the HAL in `@BeforeEach` with `assertTrue(HAL.initialize(500, 0))`.
- Construct the subsystem **directly** with sim IOs, or anonymous no-op IOs (`new GyroIO() {}`) for
  pieces the test doesn't exercise. Don't go through `<Name>Constants.get()` — that depends on the
  global mode.
- Enable the robot through `DriverStationSim` when the code under test only acts while enabled.
- Drive commands with `frc.robot.TestUtil.runTest(command, durationSeconds, subsystem)`; it runs
  `initialize`/`execute`/`isFinished`/`end` and the subsystem's `periodic()` on a real 20 ms loop.
- Compare floating-point and unit values with a `DELTA` tolerance, converting units explicitly with
  `.in(Unit)`.
- Give assertions a failure message that says what was expected in plain words.

## What to test

- New `frc.lib` utilities with pure logic (edge detection, geometry, tunables): plain unit tests.
- New IO sims, mechanisms, and subsystems: a sim-backed test showing the command reaches its goal.
- Bug fixes: a test that fails before the fix.

Keep tests fast (well under a few seconds each); they run in CI on every push.
