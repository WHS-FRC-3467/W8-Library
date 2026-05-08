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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;

import edu.wpi.first.units.measure.Angle;

import frc.lib.io.absoluteencoder.AbsoluteEncoderIO;
import frc.lib.io.motor.MotorIO;
import frc.lib.util.caliko.FabrikBone2D;
import frc.lib.util.caliko.FabrikChain2D;
import frc.lib.util.caliko.utils.Vec2f;

import lombok.Getter;

import java.util.List;

public abstract class DoubleJointedArmMechanism<
        A extends MotorIO,
        B extends AbsoluteEncoderIO,
        C extends MotorIO,
        D extends AbsoluteEncoderIO> {
    @Getter private final ArmJointMechanism<A, B> upperArm;
    @Getter private final ArmJointMechanism<C, D> lowerArm;
    private final DoubleJointedArmVisualizer visualizer;

    private final double upperLength;
    private final double lowerLength;

    private static final Vec2f RIGHT = new Vec2f(1, 0);

    public DoubleJointedArmMechanism(
            ArmJointMechanism<A, B> upperArm, ArmJointMechanism<C, D> lowerArm, String name) {
        this.upperArm = upperArm;
        this.lowerArm = lowerArm;
        this.upperLength = upperArm.characteristics.armLength().in(Feet);
        this.lowerLength = lowerArm.characteristics.armLength().in(Feet);
        visualizer =
                new DoubleJointedArmVisualizer(
                        name, upperArm.characteristics, lowerArm.characteristics);
    }

    public Angle chain2deg(FabrikChain2D chain, int num) {
        var end = chain.getBone(num).getEndLocation();
        var start = chain.getBone(num).getStartLocation();
        var run = end.x - start.x;
        var rise = end.y - start.y;
        System.out.println(Integer.toString(num) + ": " + Double.toString(Math.atan(rise / run)));
        return Degrees.of(Math.atan(rise / run));
    }

    public List<Angle> inverseKimatics(Vec2f target) {
        FabrikChain2D chain = new FabrikChain2D();
        FabrikBone2D base = new FabrikBone2D(new Vec2f(), RIGHT, (float) lowerLength);
        chain.addBone(base);
        chain.addConsecutiveBone(RIGHT, (float) upperLength);

        chain.solveForTarget(target);
        return List.of(chain2deg(chain, 0), chain2deg(chain, 1));
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
