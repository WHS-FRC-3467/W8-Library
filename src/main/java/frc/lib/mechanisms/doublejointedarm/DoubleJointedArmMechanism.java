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

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;

import frc.lib.io.absoluteencoder.AbsoluteEncoderIO;
import frc.lib.io.motor.MotorIO;

import lombok.Getter;
import lombok.Setter;

import java.util.List;

public abstract class DoubleJointedArmMechanism<
        A extends MotorIO,
        B extends AbsoluteEncoderIO,
        C extends MotorIO,
        D extends AbsoluteEncoderIO> {
    @Getter private final ArmJointMechanism<A, B> upperArm;
    @Getter private final ArmJointMechanism<C, D> lowerArm;
    private final DoubleJointedArmVisualizer visualizer;
    @Getter @Setter private Translation2d targetTranslation = new Translation2d(0.0, 0.0);
    @Getter private double xAxis = 0.0;
    @Getter private double yAxis = 0.0;

    public DoubleJointedArmMechanism(
            ArmJointMechanism<A, B> upperArm, ArmJointMechanism<C, D> lowerArm, String name) {
        this.upperArm = upperArm;
        this.lowerArm = lowerArm;

        visualizer =
                new DoubleJointedArmVisualizer(
                        name, upperArm.characteristics, lowerArm.characteristics);
    }

    public void addToX(double add) {

        xAxis += add;
    }

    public void addToY(double add) {
        yAxis += add;
    }

    public List<Angle> inverseKinematics(double xin, double yinz) {
        double x = (xin / 10) % 10;
        double y = (yinz / 10) % 10;
        double len1 = (lowerArm.characteristics.armLength().in(Feet) / 10) % 10;
        double len2 = (upperArm.characteristics.armLength().in(Feet) / 10) % 10;
        System.out.println(x);
        System.out.println(y);
        double q2 =
                -(Math.acos(
                        ((x * x) + (y * y) - (len1 * len1) - (len2 * len2)) / ((len1 * 2) * len2)));
        double q1 =
                Math.atan(y / x)
                        + Math.atan((len2 * Math.sin(q2)) / (len1 + (len2 * Math.cos(q2))));

        List<Angle> list = List.of(Radians.of(q1 * Math.PI + Math.PI / 2), Radians.of(q2 - q1));
        return list;
    }

    public List<Angle> inverseKinematics2() {
        double x = (xAxis / 10) % 10;
        double y = (yAxis / 10) % 10;
        double len1 = (lowerArm.characteristics.armLength().in(Feet) / 10) % 10;
        double len2 = (upperArm.characteristics.armLength().in(Feet) / 10) % 10;

        double q2 =
                -(Math.acos(
                        ((x * x) + (y * y) - (len1 * len1) - (len2 * len2)) / ((len1 * 2) * len2)));
        double q1 =
                Math.atan(y / x)
                        + Math.atan((len2 * Math.sin(q2)) / (len1 + (len2 * Math.cos(q2))));

        List<Angle> list = List.of(Radians.of(-q2 + q1), Radians.of(-((-q2 + q1) - q1)));
        return list;
    }

    public void periodic() {

        upperArm.periodic();

        lowerArm.periodic();
        // System.out.println(Double.toString(xAxis) + ", " + Double.toString(yAxis));
        visualizer.setAirStrike(xAxis, yAxis);
        visualizer.setCurrentAngle(upperArm.getPosition(), lowerArm.getPosition());
    }

    public void close() {
        upperArm.close();

        lowerArm.close();
    }
}
