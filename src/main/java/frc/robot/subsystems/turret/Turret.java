/*
 * Copyright (C) 2025 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;

public class Turret extends SubsystemBase implements AutoCloseable {

    private final RobotState robotState = RobotState.getInstance();

    private final RotaryMechanism rotaryIO;
    private final FlywheelMechanism flywheelIO;

    public Turret(RotaryMechanism rotaryIO, FlywheelMechanism flywheelIO)
    {
        this.rotaryIO = rotaryIO;
        this.flywheelIO = flywheelIO;
    }

    private Command setTurretPosition(Supplier<Angle> angle)
    {
        return this.run(() -> rotaryIO.runPosition(angle.get(), TurretConstants.CRUISE_VELOCITY,
            TurretConstants.ACCELERATION, TurretConstants.JERK, PIDSlot.SLOT_0));
    }

    public Command moveTurretRobotRelative(Supplier<Rotation2d> robotRelativeHeading)
    {
        return setTurretPosition(() -> Radians.of(robotRelativeHeading.get().getRadians()));
    }

    public Command moveTurretFieldRelative(
        Drive drive,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        Supplier<Rotation2d> fieldRelativeHeadingSupplier)
    {
        return Commands.parallel(
            DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier,
                fieldRelativeHeadingSupplier),
            moveTurretRobotRelative(
                () -> fieldRelativeHeadingSupplier.get()
                    .plus(robotState.getRotation().unaryMinus())));
    }

    @Override
    public void periodic()
    {
        rotaryIO.periodic();
        flywheelIO.periodic();

        var currentRobotHeading = robotState.getEstimatedPose().getRotation();

        // Robot relative
        var currentTurretHeading = Rotation2d.fromRadians(rotaryIO.getPosition().in(Radians));
        Logger.recordOutput("Turret/Orientation", currentTurretHeading.plus(currentRobotHeading));
    }

    @Override
    public void close()
    {
        rotaryIO.close();
        flywheelIO.close();
    }
}
