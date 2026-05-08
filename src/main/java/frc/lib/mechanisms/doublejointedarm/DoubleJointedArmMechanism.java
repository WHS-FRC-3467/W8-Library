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
package frc.lib.mechanisms.doublejointedarm;

import frc.lib.io.absoluteencoder.AbsoluteEncoderIO;
import frc.lib.io.motor.MotorIO;

import lombok.Getter;

public abstract class DoubleJointedArmMechanism<
        A extends MotorIO,
        B extends AbsoluteEncoderIO,
        C extends MotorIO,
        D extends AbsoluteEncoderIO> {
    @Getter private final ArmJointMechanism<A, B> upperArm;
    @Getter private final ArmJointMechanism<C, D> lowerArm;
    private final DoubleJointedArmVisualizer visualizer;

    public DoubleJointedArmMechanism(
            ArmJointMechanism<A, B> upperArm, ArmJointMechanism<C, D> lowerArm, String name) {
        this.upperArm = upperArm;
        this.lowerArm = lowerArm;

        visualizer =
                new DoubleJointedArmVisualizer(
                        name, upperArm.characteristics, lowerArm.characteristics);
    }

    public void periodic() {

        upperArm.periodic();

        lowerArm.periodic();

        visualizer.setCurrentAngle(upperArm.getPosition(), lowerArm.getPosition());
    }

    public void close() {
        upperArm.close();

        lowerArm.close();
    }
}
