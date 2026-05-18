/*
 * Copyright (C) 2026 Windham Windup
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
package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.doublejointedarm.DoubleJointedArmMechanism;
import frc.lib.util.LoggerHelper;

import java.util.function.DoubleSupplier;

public class Arm extends SubsystemBase implements AutoCloseable {
    public final DoubleJointedArmMechanism<?, ?, ?, ?> io;

    public Arm(DoubleJointedArmMechanism<?, ?, ?, ?> io) {
        this.io = io;
    }

    @Override
    public void periodic() {

        io.periodic();
        super.periodic();
        LoggerHelper.recordCurrentCommand(this.getName(), this);
    }

    @Override
    public void close() {
        io.close();
    }

    public Command moveLowerBy(AngularVelocity pos, boolean up) {

        return this.runOnce(() -> io.getLowerArm().runVelocity(pos, PIDSlot.SLOT_0));
    }

    public Command moveUpperBy(AngularVelocity pos, boolean up) {

        return this.runOnce(() -> io.getUpperArm().runVelocity(pos, PIDSlot.SLOT_0));
    }

    public Command testUpper() {
        return this.runOnce(
                () -> io.getUpperArm().runUnprofiledPosition(Radians.of(1.0), PIDSlot.SLOT_0));
    }

    public Command stopUpper() {
        return this.runOnce(() -> io.getUpperArm().runBrake());
    }

    public Command stopLower() {
        return this.runOnce(() -> io.getLowerArm().runBrake());
    }

    public Command changeTarget(DoubleSupplier x, DoubleSupplier y) {
        return this.run(() -> io.addToTarget(x.getAsDouble(), y.getAsDouble()));
    }

    public Command runIK() {
        return this.runOnce(() -> io.runTranslation(PIDSlot.SLOT_0));
    }
}
