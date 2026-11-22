# Robot Control Structure (Commands v3 + StateMachine)

How robot code on this library is organized for the 2027 season. It uses WPILib Commands v3
(`org.wpilib.command3`) and its `StateMachine` class, on WPILib `2027.0.0-alpha-8`.

The worked example is the Perry code in `frc.robot`. Read
[Superstructure.java](../src/main/java/frc/robot/subsystems/superstructure/Superstructure.java)
and [Flywheel.java](../src/main/java/frc/robot/subsystems/shooter/Flywheel.java) alongside this page.

## The idea

Every part of the robot is always in exactly one named state, and code changes what the robot is
doing by **requesting a state**, never by commanding a motor directly.

There are three layers. Each layer only talks to the one below it.

| Layer | Example | Owns hardware? | What it does |
| --- | --- | --- | --- |
| Bindings and autos | `RobotContainer`, auto routines | No | Request robot states: `superstructure.score()` |
| Superstructure | `Superstructure` | No | One robot-level state machine. Each state sets the goal of every mechanism. |
| Mechanisms | `Flywheel`, `Hood`, `IntakeSlide`, ... | Yes, one actuator each | One small state machine per mechanism. Each state is one hardware action. |

The drivetrain is the exception. It stays a normal v3 mechanism with a default drive command,
because driving is continuous joystick control and does not fit named states.

## Rules

1. **One `Mechanism` per actuator.** A hood and a flywheel are two mechanisms, even though they are
   one assembly. This lets the scheduler own them separately.
2. **Every mechanism has a `Goal` enum and a `GoalStateMachine`.** The mechanism's only public
   control method is `request(Goal)`. Hardware calls live inside the state bodies and nowhere else.
3. **Mechanisms report readiness as public `Trigger` fields**, such as `flywheel.atShootSpeed` or
   `slide.isRollerSafe`.
4. **Only the superstructure calls `request()` on mechanisms.** Bindings and autos call the
   superstructure's command methods (`intake()`, `score()`, `stow()`, `eject()`).
5. **Every superstructure state sets the goal of every mechanism.** Don't rely on what the last
   state left behind. This is what makes it safe to jump from any state to any other.
6. **Interlocks go in the superstructure.** Waiting inside a state (`c.waitUntil(slide.isRollerSafe)`)
   covers ordering within a state. A transition (`addTransition(AIM, SHOOT, readyToShoot)`) covers
   moving on by itself. If a state must not be requested directly, don't give it a command method.
   `SHOOT` is the example: the only way in is through `AIM`.
7. **Anything that is not a goal is a normal command.** Homing, SysId, and manual jogging are
   written with `mechanism.run(...)`. While one runs it takes the mechanism from its state machine,
   and the state machine starts again by itself when the command ends. See `IntakeSlide.home()`.

## Writing a mechanism

```java
public class IntakeRoller implements Mechanism {
    public enum Goal { STOP, INTAKE, EJECT }

    private final GoalStateMachine<Goal> goals;

    public IntakeRoller(FlywheelActuator<?> io) {
        goals = new GoalStateMachine<>("Intake Roller", this, Goal.class, Goal.STOP);
        goals.addState(Goal.STOP, c -> io.runBrake());
        goals.addState(Goal.INTAKE, c -> io.runCurrent(Amps.of(80)));
        goals.addState(Goal.EJECT, c -> io.runCurrent(Amps.of(-40)));
        goals.start();

        Scheduler.getDefault().addPeriodic(io::periodic);
    }

    public void request(Goal goal) {
        goals.request(goal);
    }
}
```

- A state body runs once when the state is entered. After it returns, the state holds the
  mechanism until the machine moves on. A body can also wait (`c.waitUntil`, `c.wait`) to do a
  short sequence.
- `start()` throws if any enum value has no state, so a goal can't be forgotten.
- There is no `periodic()` in v3. Register input updates with `Scheduler.getDefault().addPeriodic`.

## Writing the superstructure

```java
goals.addState(Goal.INTAKE, c -> {
    indexer.request(Indexer.Goal.STOP);
    tower.request(Tower.Goal.STOP);
    flywheel.request(Flywheel.Goal.IDLE);
    hood.request(Hood.Goal.STOW);

    slide.request(IntakeSlide.Goal.EXTEND);
    roller.request(IntakeRoller.Goal.STOP);
    c.waitUntil(slide.isRollerSafe);
    roller.request(IntakeRoller.Goal.INTAKE);
});

goals.addTransition(Goal.AIM, Goal.SHOOT, readyToShoot);
```

Bindings then stay one line each:

```java
controller.rightTrigger().onTrue(superstructure.score()).onFalse(superstructure.stow());
```

An auto is a coroutine that requests states and waits on triggers:

```java
Command.noRequirements(c -> {
    c.await(superstructure.score());
    c.waitUntil(superstructure.isShooting);
    c.wait(Seconds.of(1.5));
    c.await(superstructure.stow());
}).named("Shoot Preload");
```

## What `GoalStateMachine` adds on top of WPILib's `StateMachine`

[GoalStateMachine](../src/main/java/frc/lib/statemachine/GoalStateMachine.java) is a thin wrapper.
It builds a normal WPILib `StateMachine` and handles four things that are easy to get wrong.

1. **Lifecycle.** A WPILib `StateMachine` has no requirements, so it can't be a default command and
   nothing restarts it. The wrapper installs a default command on the mechanism that runs the
   machine as its child. The result: the machine stops when the robot is disabled, restarts in its
   initial state on enable, and resumes after an override command ends.
2. **States that hold.** If a state's command finishes and no transition is set, a WPILib
   `StateMachine` exits. The wrapper parks every state after its body so this can't happen.
3. **Requests.** One shared transition sends the machine to whatever state was last requested, so
   states don't need a hand-written transition to every other state.
4. **Re-entry.** WPILib transitions fire only on the rising edge of their condition, and the
   machine remembers the last value it saw even while in other states. A condition that fired once
   and is still true the next time the state is entered never fires again. The wrapper forces one
   false reading on every state entry to reset that. `GoalStateMachineTest` covers this case.

## Things to know

- **Requests take a loop or two to travel.** A superstructure request reaches a mechanism on the
  next scheduler run, and a v3 `Trigger` only updates when the scheduler polls it at the start of
  each run. Expect 20 to 60 ms between a button press and a motor command, and the same for a
  readiness trigger to be seen by the superstructure. Tests must not assert on the very next loop.
- **Don't read a `Trigger` you just created.** It reads false until the scheduler has polled it.
  Create triggers once in constructors.
- **Never name a method `idle()` on a mechanism.** `Mechanism.idle()` already exists in WPILib and
  overriding it by accident changes the mechanism's fallback command. The superstructure uses
  `stow()` for this reason.
- **`SHOOT` latches.** Once in `SHOOT` the robot keeps feeding even if the flywheel drops out of
  tolerance. Add `addTransition(SHOOT, AIM, readyToShoot.negate())` if we want it to pause feeding.
- **Tuning values are read on state entry.** A `LoggedTunableNumber` change applies the next time
  the state is entered, not while it is held.
- **Logging.** Each machine logs `<name>/ActiveState` and `<name>/RequestedState`. Plot
  `Superstructure/ActiveState` first when reviewing a match.

## When not to use this

Use a plain coroutine command instead of a state machine when the behavior is a one-off sequence
with a clear end and nothing else needs to ask what state it is in. Auto routines, homing, and
characterization are the usual cases.

## Status

- Built and tested against WPILib `2027.0.0-alpha-8`. Commands v3 and `StateMachine` are pre-release
  and can change before kickoff. WPILib `main` already adds `StateMachine.setPriority`, which
  alpha 8 does not have.
- `GoalStateMachineTest` covers the wrapper. `SuperstructureTest` runs IDLE, INTAKE, AIM, SHOOT, and
  back to IDLE against the simulated actuators.
- Shot speed and hood angle are single tunable numbers in the example. A real robot would compute
  them from distance, inside the `SHOOT` state of `Flywheel` and `Hood`.
