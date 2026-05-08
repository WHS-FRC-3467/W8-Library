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
package frc.robot.subsystems.doublejointedarm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.doublejointedarm.DoubleJointedArmMechanism;
import frc.lib.util.caliko.utils.Vec2f;

public class Arm extends SubsystemBase implements AutoCloseable {
    private final DoubleJointedArmMechanism<?, ?, ?, ?> io;

    public Arm(DoubleJointedArmMechanism<?, ?, ?, ?> io) {
        this.io = io;
    }

    public Command kimaticsMotion(float x, float y) {
        var vec = new Vec2f(x, y);
        var list = io.inverseKimatics(vec);
        return this.runOnce(
                () -> {
                    io.getLowerArm().runUnprofiledPosition(list.get(0), PIDSlot.SLOT_0);
                    io.getUpperArm().runUnprofiledPosition(list.get(1), PIDSlot.SLOT_0);
                });
    }

    public Command testCommand() {
        var list = io.inverseKimatics(new Vec2f(8, 7));
        System.out.println(list.get(0));
        return this.moveUpper(list.get(1), true).andThen(this.moveLower(list.get(0), true));
    }

    public Command moveUpper(Angle pos, boolean up) {
        return this.runOnce(
                () ->
                        io.getUpperArm()
                                .runUnprofiledPosition(
                                        up
                                                ? io.getUpperArm().getPosition().plus(pos)
                                                : io.getUpperArm().getPosition().minus(pos),
                                        PIDSlot.SLOT_0));
    }

    public Command moveLower(Angle pos, boolean up) {

        return this.runOnce(
                () ->
                        io.getLowerArm()
                                .runUnprofiledPosition(
                                        up
                                                ? io.getLowerArm().getPosition().plus(pos)
                                                : io.getLowerArm().getPosition().minus(pos),
                                        PIDSlot.SLOT_0));
    }

    @Override
    public void periodic() {
        io.periodic();
        super.periodic();
    }

    @Override
    public void close() {
        io.close();
    }
}
