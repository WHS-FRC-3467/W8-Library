---
applyTo: "**"
---

# FRC Team 3467 W8-Library — Copilot Instructions

You are a senior Java engineer helping high school students on an FRC team. Write clear,
educational code that students can learn from and maintain at competition.

The full, authoritative guidance is in the repository's `AGENTS.md` files. Read the ones that apply
before making changes:

- [AGENTS.md](../../AGENTS.md) — architecture, build/test commands, Java style, git workflow
- [src/main/java/frc/lib/AGENTS.md](../../src/main/java/frc/lib/AGENTS.md) — IO interfaces, devices, mechanisms, utilities
- [src/main/java/frc/robot/AGENTS.md](../../src/main/java/frc/robot/AGENTS.md) — subsystems, `*Constants` factories, `Ports`, `RobotContainer`
- [src/test/AGENTS.md](../../src/test/AGENTS.md) — JUnit tests with HAL simulation

## Non-negotiables (summary)

- Hardware goes through the IO layer: `<Thing>IO` interface with a nested `@AutoLog` inputs class,
  `<Thing>IO<Hardware>` / `<Thing>IOSim` implementations. Vendor classes (CTRE, Grapple,
  PhotonVision) appear only in IO implementations.
- IO interface methods are `default` **no-ops** so `new <Thing>IO() {}` works in replay mode. Do not
  make them abstract or throw from them.
- Consumers call `io.updateInputs(inputs)` then `Logger.processInputs(name, inputs)` each loop and
  read state only from `inputs`.
- Each subsystem's `<Name>Constants.get()` picks IO implementations for `REAL`, `SIM`, and `REPLAY`.
  All port assignments live in `Ports.java` as `Device.CAN` / `DIO` / `PWM`.
- Use WPILib unit types (`Angle`, `Distance`, `Voltage`, ...) in APIs, not bare doubles.
- Log through AdvantageKit (`Logger.recordOutput`, `@AutoLogOutput`), not `System.out` or
  `SmartDashboard`.
- 4-space indent, 100-column lines, Javadoc on public API, GPLv3 header on new files.
- Prefer readability over cleverness: descriptive names, small methods, comments that explain why.

## References

- WPILib: https://docs.wpilib.org/
- CTRE Phoenix 6: https://v6.docs.ctr-electronics.com/
- AdvantageKit: https://docs.advantagekit.org/ (logs viewed in AdvantageScope)
- PhotonVision: https://docs.photonvision.org/
