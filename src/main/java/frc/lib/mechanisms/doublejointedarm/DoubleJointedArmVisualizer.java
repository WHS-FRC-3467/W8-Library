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
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.derive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.DistanceUnit;
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

    private final LoggedMechanismLigament2d axisXX;
    private final LoggedMechanismLigament2d axisXY;

    private final LoggedMechanismLigament2d axisYY;
    private final LoggedMechanismLigament2d axisYX;

    private final String name;

    private DistanceUnit visualizerUnit;

    private static final Color8Bit LINE_COLOR = new Color8Bit(15, 12, 22);

    public DoubleJointedArmVisualizer(
            String name,
            JointCharacteristics upperCharacteristics,
            JointCharacteristics lowerCharacteristics) {
        this.name = name;
        this.visualizerUnit =
                derive(Meters)
                        .aggregate(
                                (upperCharacteristics.armLength().in(Meters)
                                                + lowerCharacteristics.armLength().in(Meters))
                                        / 2)
                        .named("visualizerUnit")
                        .symbol("vu")
                        .make();

        mechanism = new LoggedMechanism2d(8, 8, new Color8Bit(Color.kBlack));
        LoggedMechanismRoot2d root = mechanism.getRoot(name + " root", 4, 4);

        upperMeasured =
                new LoggedMechanismLigament2d(
                        "Upper" + name + "Measured",
                        upperCharacteristics.armLength().in(visualizerUnit),
                        upperCharacteristics.startingAngle().in(Degrees)
                                + lowerCharacteristics.startingAngle().in(Degrees),
                        2.5,
                        new Color8Bit(Color.kAquamarine));

        lowerMeasured =
                new LoggedMechanismLigament2d(
                        "Lower" + name + "Measured",
                        upperCharacteristics.armLength().in(visualizerUnit),
                        lowerCharacteristics.startingAngle().in(Degrees),
                        2.5,
                        new Color8Bit(Color.kOrange));

        axisXX = new LoggedMechanismLigament2d("axis" + name + "XX", 0, 0, 1, LINE_COLOR);

        axisXY =
                new LoggedMechanismLigament2d(
                        "axis" + name + "XY", 0, 90, 2.5, new Color8Bit(Color.kRed));
        axisYY = new LoggedMechanismLigament2d("axis" + name + "YY", 0, 90, 1, LINE_COLOR);
        axisYX =
                new LoggedMechanismLigament2d(
                        "axis" + name + "YX", 0, -90, 2.5, new Color8Bit(Color.kRed));

        root.append(axisXX);
        axisXX.append(axisXY);

        root.append(axisYY);
        axisYY.append(axisYX);
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

    public void setTargetPosition(Translation2d translation) {
        double x = translation.getX();
        double y = translation.getY();

        axisXX.setLength(x);
        axisXY.setLength(y);

        axisYY.setLength(y);
        axisYX.setLength(x);

        update();
    }
}
