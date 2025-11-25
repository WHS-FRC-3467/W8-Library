// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.revRotary;

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

public class RevRotary extends SubsystemBase {

    private final RotaryMechanism io;

    private static final LoggedTunableNumber STOW_SETPOINT =
        new LoggedTunableNumber("REV STOW", 0.0);
    private static final LoggedTunableNumber RAISED_SETPOINT =
        new LoggedTunableNumber("REV RAISED", 90);

    @RequiredArgsConstructor
    @Getter
    public enum Setpoint {
        STOW(STOW_SETPOINT),
        RAISED(RAISED_SETPOINT);

        private final LoggedTunableNumber tunableNumber;

        public Angle getSetpoint()
        {
            return Degrees.of(tunableNumber.get());
        }
    }


    public RevRotary(RotaryMechanism io)
    {
        this.io = io;

        setSetpoint(RevRotaryConstants.DEFAULT_SETPOINT).ignoringDisable(true).schedule();
    }

    @Override
    public void periodic()
    {
        LoggerHelper.recordCurrentCommand(RevRotaryConstants.NAME, this);
        io.periodic();
        // robotstate.setRotaryPose(io.getPoseSupplier().get());
    }

    public Command setSetpoint(Setpoint setpoint)
    {
        return this.runOnce(
            () -> io.runPosition(setpoint.getSetpoint(),
                PIDSlot.SLOT_0))
            .withName("Go To " + setpoint.toString() + " Setpoint");
    };

    public boolean nearGoal(Angle targetPosition)
    {
        return io.nearGoal(targetPosition, RevRotaryConstants.TOLERANCE);
    }

    public Command waitUntilGoalCommand(Angle position)
    {
        return Commands.waitUntil(() -> {
            return nearGoal(position);
        });
    }

    public Command setGoalCommandWithWait(Setpoint setpoint)
    {
        return waitUntilGoalCommand(setpoint.getSetpoint())
            .deadlineFor(setSetpoint(setpoint))
            .withName("Go To " + setpoint.toString() + " Setpoint with wait");
    }

    public AngularVelocity getVelocity()
    {
        return io.getVelocity();
    }

    public void close()
    {
        io.close();
    }

}
