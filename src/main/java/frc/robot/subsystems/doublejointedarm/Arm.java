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

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.doublejointedarm.DoubleJointedArmMechanism;
import frc.lib.util.AlwaysTunableNumber;

import java.util.List;

public class Arm extends SubsystemBase implements AutoCloseable {
    private final DoubleJointedArmMechanism<?, ?, ?, ?> io;
    private final AlwaysTunableNumber tunableX = new AlwaysTunableNumber("tunableX", 0.45);

    private final AlwaysTunableNumber tunableY = new AlwaysTunableNumber("tunableY", 0.30);
    public Angle lowerAngle = Radians.of(0);
    public Angle upperAngle = Radians.of(0);

    public Arm(DoubleJointedArmMechanism<?, ?, ?, ?> io) {
        this.io = io;
    }

    public Command moveUpperBy(Angle pos, boolean up) {
        return this.runOnce(
                () ->
                        io.getUpperArm()
                                .runUnprofiledPosition(
                                        up
                                                ? io.getUpperArm().getPosition().plus(pos)
                                                : io.getUpperArm().getPosition().minus(pos),
                                        PIDSlot.SLOT_0));
    }

    public void setAngles() {
        List<Angle> list = io.inverseKinematics2();
        lowerAngle = list.get(0);
        upperAngle = list.get(1);
    }

    public Command setAnglesCommand() {
        return this.runOnce(() -> setAngles());
    }

    public Command yAxis(double yinz) {
        return this.run(() -> io.addToY(yinz));
    }

    public Command xAxis(double xinz) {
        return this.run(() -> io.addToX(xinz));
    }

    public Command moveUpperTo(Angle pos) {
        return this.runOnce(() -> io.getUpperArm().runUnprofiledPosition(pos, PIDSlot.SLOT_0));
    }

    public Command moveLowerTo(Angle pos) {
        return this.runOnce(() -> io.getLowerArm().runUnprofiledPosition(pos, PIDSlot.SLOT_0));
    }

    public List<Angle> inverseKinematics(double xin, double yinz) {

        return io.inverseKinematics(xin, yinz);
    }

    public List<Angle> inverseKinematicsTunable() {

        return io.inverseKinematics2();
    }

    public String inverseKinematicsString(double xin, double yinz) {

        var kit = io.inverseKinematics(xin, yinz);
        String out = "";
        for (var angle : kit) {
            out += ", " + Double.toString(angle.in(Radians));
        }
        return out;
    }

    public Command moveArmsTo(List<Angle> poses) {
        if (poses.size() > 2) {
            return Commands.none();
        } else {
            return moveLowerTo(poses.get(0)).andThen(moveUpperTo(poses.get(1)));
        }
    }

    public void moveArms() {
        io.getLowerArm().runUnprofiledPosition(lowerAngle, PIDSlot.SLOT_0);
        io.getUpperArm().runUnprofiledPosition(upperAngle, PIDSlot.SLOT_0);
    }

    public Command moveLowerBy(Angle pos, boolean up) {

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
