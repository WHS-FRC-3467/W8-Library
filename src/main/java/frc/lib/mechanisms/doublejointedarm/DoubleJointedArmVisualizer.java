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
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import frc.lib.mechanisms.doublejointedarm.ArmJointMechanism.JointCharacteristics;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class DoubleJointedArmVisualizer {
    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismLigament2d upperMeasured;
    private final LoggedMechanismLigament2d lowerMeasured;

    private final LoggedMechanismLigament2d airStrike;

    private final String name;

    private final double upperArmLength;
    private final double lowerArmLength;

    public DoubleJointedArmVisualizer(
            String name,
            JointCharacteristics upperCharacteristics,
            JointCharacteristics lowerCharacteristics) {
        this.name = name;
        upperArmLength = upperCharacteristics.armLength().in(Feet);
        lowerArmLength = lowerCharacteristics.armLength().in(Feet);
        double averageLength = (upperArmLength + lowerArmLength) / 2;
        mechanism = new LoggedMechanism2d(90, 90, new Color8Bit(Color.kBlack));
        LoggedMechanismRoot2d root = mechanism.getRoot(name + " root", 45, 45);

        upperMeasured =
                new LoggedMechanismLigament2d(
                        "Upper" + name + "Measured",
                        upperArmLength * 20,
                        upperCharacteristics.startingAngle().in(Degrees),
                        2.5,
                        new Color8Bit(Color.kAquamarine));

        lowerMeasured =
                new LoggedMechanismLigament2d(
                        "Lower" + name + "Measured",
                        lowerArmLength * 20,
                        lowerCharacteristics.startingAngle().in(Degrees),
                        2.5,
                        new Color8Bit(Color.kOrange));

        airStrike =
                new LoggedMechanismLigament2d("CoolAssThang", 0, 0, 5, new Color8Bit(Color.kRed));

        root.append(airStrike);
        root.append(lowerMeasured);

        lowerMeasured.append(upperMeasured);
    }

    private void update() {
        SmartDashboard.putData("Mechanism Visualizers/" + name + " Visualizer", mechanism);
    }

    /**
     * Sets the current measured angle of the doubleJointedArm mechanism.
     *
     * @param upperAngle The measured angle of the upper arm to display
     * @param lowerAngle The measured angle of the lower arm to display
     */
    public void setCurrentAngle(Angle upperAngle, Angle lowerAngle) {
        upperMeasured.setAngle(Rotation2d.fromRadians(upperAngle.in(Radians)));
        lowerMeasured.setAngle(Rotation2d.fromRadians(lowerAngle.in(Radians)));

        update();
    }

    public void setAirStrike(double x, double y) {
        Angle slope = Radians.of(Math.atan2(y, x));
        double length = Math.sqrt(y * y + x * x) * 20;
        airStrike.setAngle(slope);
        airStrike.setLength(length);
        update();
    }
}
