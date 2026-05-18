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
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

import frc.lib.io.absoluteencoder.AbsoluteEncoderIO;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.util.AlwaysTunableNumber;

import lombok.Getter;

public abstract class DoubleJointedArmMechanism<
        A extends MotorIO,
        B extends AbsoluteEncoderIO,
        C extends MotorIO,
        D extends AbsoluteEncoderIO> {
    @Getter private final ArmJointMechanism<A, B> upperArm;
    @Getter private final ArmJointMechanism<C, D> lowerArm;
    private final DoubleJointedArmVisualizer visualizer;

    private Translation2d target = new Translation2d(1.0, 1.0);

    private AlwaysTunableNumber num = new AlwaysTunableNumber("i dunno", 0);

    public void addToTarget(double x, double y) {
        this.target = this.target.plus(new Translation2d(x, y));
    }

    public DoubleJointedArmMechanism(
            ArmJointMechanism<A, B> upperArm, ArmJointMechanism<C, D> lowerArm, String name) {
        this.upperArm = upperArm;
        this.lowerArm = lowerArm;

        visualizer =
                new DoubleJointedArmVisualizer(
                        name, upperArm.characteristics, lowerArm.characteristics);
    }

    public void runTranslation(Translation2d translation, Pair<PIDSlot, PIDSlot> pid) {
        var kinematicsResults = inverseKinematics(translation.getX(), translation.getY());

        AngularVelocity cruiseVelocity = RadiansPerSecond.of(1.0);
        AngularAcceleration acceleration = RadiansPerSecondPerSecond.of(1.0);
        lowerArm.runPosition(
                kinematicsResults.getFirst(), pid.getFirst(), cruiseVelocity, acceleration);
        upperArm.runPosition(
                kinematicsResults.getSecond(), pid.getSecond(), cruiseVelocity, acceleration);
    }

    public void runTranslation(Translation2d translation, PIDSlot pid) {
        runTranslation(translation, Pair.of(pid, pid));
    }

    public void runTranslation(PIDSlot pid) {
        runTranslation(target, Pair.of(pid, pid));
    }

    private static final double PI2 = Math.PI / 2;

    private Pair<Angle, Angle> inverseKinematics(double xin, double yin) {

        double x = (Math.abs(xin) / 10) % 10;
        double y = (yin / 10) % 10;
        double len1 = (lowerArm.characteristics.armLength().in(Feet) / 10) % 10;
        double len2 = (upperArm.characteristics.armLength().in(Feet) / 10) % 10;

        double q2 =
                -(Math.acos(
                        ((x * x) + (y * y) - (len1 * len1) - (len2 * len2)) / ((len1 * 2) * len2)));
        double q1 =
                Math.atan(y / x)
                        + Math.atan((len2 * Math.sin(q2)) / (len1 + (len2 * Math.cos(q2))));

        double q1out = -q2 + q1;
        double q2out = -((-q2 + q1) - q1);

        if (xin < 0.0) {
            q1out = (PI2 - q1out) + PI2;
            q2out = -q2out;
        }

        return Pair.of(Radians.of(q1out), Radians.of(q2out));
    }

    public void periodic() {
        upperArm.periodic();
        lowerArm.periodic();

        visualizer.setCurrentAngle(upperArm.getPosition(), lowerArm.getPosition());
        visualizer.setTargetPosition(target);
    }

    public void close() {
        upperArm.close();

        lowerArm.close();
    }
}
