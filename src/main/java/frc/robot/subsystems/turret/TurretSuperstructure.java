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
import static edu.wpi.first.units.Units.RotationsPerSecond;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.devices.BeamBreak;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import frc.lib.util.RisingEdge;
import frc.robot.RobotState;

public class TurretSuperstructure extends SubsystemBase implements AutoCloseable {

    private final RobotState robotState = RobotState.getInstance();

    private final BeamBreak indexerBeamBreak;

    private final RotaryMechanism rotaryIO;
    private final RotaryMechanism indexerIO;
    private final FlywheelMechanism flywheelIO;

    public TurretSuperstructure(RotaryMechanism rotaryIO, RotaryMechanism indexerIO,
        FlywheelMechanism flywheelIO,
        BeamBreak indexerBeamBreak)
    {
        this.rotaryIO = rotaryIO;
        this.indexerIO = indexerIO;
        this.flywheelIO = flywheelIO;
        this.indexerBeamBreak = indexerBeamBreak;
    }

    private void runIndexer()
    {
        indexerIO.runVelocity(IndexerConstants.CRUISE_VELOCITY, IndexerConstants.ACCELERATION,
            PIDSlot.SLOT_0);
    }

    private void stopIndexer()
    {
        indexerIO.runVelocity(RotationsPerSecond.zero(), IndexerConstants.ACCELERATION,
            PIDSlot.SLOT_0);
    }

    private Command interateIndexerPosition()
    {
        return Commands.startEnd(this::runIndexer, this::stopIndexer)
            .until(RisingEdge.of(indexerBeamBreak::isBroken));
    }

    private void setTurretPosition(Angle angle)
    {
        rotaryIO.runPosition(angle, TurretConstants.CRUISE_VELOCITY,
            TurretConstants.ACCELERATION, TurretConstants.JERK, PIDSlot.SLOT_0);
    }

    private boolean turretIsAt(Angle angle)
    {
        return rotaryIO.nearGoal(angle, TurretConstants.TOLERANCE);
    }

    private Command setTurretPositionAndWait(Angle angle)
    {
        return Commands.runOnce(() -> setTurretPosition(angle))
            .andThen(Commands.waitUntil(() -> turretIsAt(angle)));
    }

    private void spinFlywheel(AngularVelocity velocity)
    {
        flywheelIO.runVelocity(velocity, FlywheelConstants.MAX_ACCELERATION, PIDSlot.SLOT_0);
    }

    private boolean flywheelIsAt(AngularVelocity velocity)
    {
        return MathUtil.isNear(
            velocity.in(RotationsPerSecond),
            flywheelIO.getVelocity().in(RotationsPerSecond),
            FlywheelConstants.TOLERANCE.in(RotationsPerSecond));
    }

    private Command spinFlywheelAndWait(AngularVelocity velocity)
    {
        return Commands.runOnce(() -> spinFlywheel(velocity))
            .andThen(Commands.waitUntil(() -> flywheelIsAt(velocity)));
    }

    public Command shoot(Angle angle)
    {
        return Commands.sequence(
            Commands.parallel(
                setTurretPositionAndWait(angle),
                spinFlywheelAndWait(FlywheelConstants.MAX_VELOCITY)),
            interateIndexerPosition(),
            Commands.runOnce(() -> spinFlywheel(RotationsPerSecond.zero())));
    }

    @Override
    public void periodic()
    {
        rotaryIO.periodic();
        flywheelIO.periodic();
        indexerIO.periodic();
        indexerBeamBreak.periodic();

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
        indexerIO.close();
    }
}
