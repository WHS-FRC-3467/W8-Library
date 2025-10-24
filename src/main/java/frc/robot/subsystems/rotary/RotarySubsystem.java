// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rotary;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggerHelper;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class RotarySubsystem extends SubsystemBase {

    private final RotaryMechanism io;

    private static final LoggedTunableNumber STOW_SETPOINT = new LoggedTunableNumber("TEST", 0.0);
    private static final LoggedTunableNumber RAISED_SETPOINT =
        new LoggedTunableNumber("RAISED", 90);

    /**
     * Predefined positions for the rotary mechanism.
     * 
     * <p>
     * These setpoints define common positions that the mechanism moves to during operation. The
     * actual angle values can be tuned live during testing using LoggedTunableNumber.
     */
    @RequiredArgsConstructor
    @SuppressWarnings("Immutable")
    @Getter
    public enum Setpoint {
        /**
         * The stowed (stored) position - where the mechanism rests safely inside the robot frame
         */
        STOW(Degrees.of(STOW_SETPOINT.get())),
        RAISED(Degrees.of(RAISED_SETPOINT.get()));

        private final Angle setpoint;
    }

    /**
     * Constructs a new RotarySubsystem.
     * 
     * <p>
     * This constructor initializes the subsystem and immediately schedules a command to move to the
     * default setpoint.
     * 
     * @param io The RotaryMechanism IO layer (real hardware, simulation, or replay)
     */
    public RotarySubsystem(RotaryMechanism io)
    {
        this.io = io;

        setSetpoint(RotarySubsystemConstants.DEFAULT_SETPOINT).ignoringDisable(true).schedule();
    }

    /**
     * Called every robot loop iteration (every 20ms by default).
     * 
     * <p>
     * This method logs which command is currently using this subsystem and updates all mechanism
     * telemetry for AdvantageKit logging.
     */
    @Override
    public void periodic()
    {
        LoggerHelper.recordCurrentCommand(RotarySubsystemConstants.NAME, this);
        io.periodic();
    }

    /**
     * Creates a command to move the mechanism to a predefined setpoint.
     * 
     * <p>
     * This command uses Motion Magic control, which creates smooth motion profiles with controlled
     * velocity and acceleration. The command completes instantly - it only starts the motion and
     * does not wait for the mechanism to reach the target.
     * 
     * @param setpoint The predefined position to move to (STOW or RAISED)
     * @return A Command that starts motion to the setpoint
     */
    public Command setSetpoint(Setpoint setpoint)
    {
        return this.runOnce(
            () -> io.runPosition(setpoint.getSetpoint(), RotarySubsystemConstants.CRUISE_VELOCITY,
                RotarySubsystemConstants.ACCELERATION, RotarySubsystemConstants.JERK,
                PIDSlot.SLOT_0))
            .withName("Go To " + setpoint.toString() + " Setpoint");
    };

    /**
     * Checks if the mechanism is near the goal position within the defined tolerance.
     * 
     * <p>
     * This is useful for determining when the mechanism has reached its target position. The
     * tolerance is defined in RotarySubsystemConstants.TOLERANCE.
     * 
     * @param targetPosition The position to check against
     * @return true if the current position is within tolerance of the target
     */
    public boolean nearGoal(Angle targetPosition)
    {
        return io.nearGoal(targetPosition, RotarySubsystemConstants.TOLERANCE);
    }

    /**
     * Creates a command that waits until the mechanism reaches the specified position.
     * 
     * <p>
     * This command finishes when nearGoal() returns true. It's useful for creating command
     * sequences where you need to wait for the mechanism to finish moving before continuing to the
     * next step.
     * 
     * @param position The position to wait for
     * @return A Command that finishes when the position is reached
     */
    public Command waitUntilGoalCommand(Angle position)
    {
        return Commands.waitUntil(() -> {
            return nearGoal(position);
        });
    }

    /**
     * Creates a command that moves to a setpoint and waits until it's reached.
     * 
     * <p>
     * This combines setSetpoint() and waitUntilGoalCommand() into a single command. The deadline
     * composition ensures that if the wait timeout occurs, both the wait and the motion command are
     * cancelled together.
     * 
     * @param setpoint The predefined position to move to and wait for
     * @return A Command that moves to the setpoint and waits until it's reached
     */
    public Command setGoalCommandWithWait(Setpoint setpoint)
    {
        return waitUntilGoalCommand(setpoint.getSetpoint())
            .deadlineFor(setSetpoint(setpoint))
            .withName("Go To " + setpoint.toString() + " Setpoint with wait");
    }

    /**
     * Gets the current velocity of the mechanism.
     * 
     * @return The current angular velocity
     */
    public AngularVelocity getVelocity()
    {
        return io.getVelocity();
    }

    /**
     * Closes and cleans up resources used by this subsystem.
     * 
     * <p>
     * This should be called when the subsystem is no longer needed, typically during robot
     * shutdown. It ensures proper cleanup of hardware resources.
     */
    public void close()
    {
        io.close();
    }
}
