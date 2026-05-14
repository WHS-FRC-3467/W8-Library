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

    private final LoggedMechanismLigament2d targetPosition0;
    private final LoggedMechanismLigament2d targetPosition1;
    private final LoggedMechanismLigament2d targetPosition2;
    private final LoggedMechanismLigament2d targetPosition3;

    private final LoggedMechanismLigament2d targetLigament00; // the price you pay for stupidity
    private final LoggedMechanismLigament2d targetLigament0;
    private final LoggedMechanismLigament2d targetLigament1;
    private final LoggedMechanismLigament2d targetLigament2;

    private final LoggedMechanismLigament2d roundXLogger;

    private final String name;

    private DistanceUnit visualizerUnit;

    private static final Color8Bit BACKROUND_COLOR = new Color8Bit(173, 167, 186);

    private static final Color8Bit LINE_COLOR = new Color8Bit(15, 12, 22);

    private static final double BULLSEYE_SIZE = 0.2;

    private static final boolean DEBUG = true;

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

        double bullseyeLength = Math.sqrt(((BULLSEYE_SIZE) * (BULLSEYE_SIZE)) * 2);
        targetPosition0 =
                new LoggedMechanismLigament2d(
                        "targetPosition0", pothag(0.5, 0.5), 0, 1, BACKROUND_COLOR);
        targetPosition1 = new LoggedMechanismLigament2d("targetPosition1", 0, 0, 1, LINE_COLOR);
        targetPosition2 = new LoggedMechanismLigament2d("targetPosition2", 0, 0, 1, LINE_COLOR);
        targetPosition3 = new LoggedMechanismLigament2d("targetPosition3", 0, 0, 1, LINE_COLOR);

        targetLigament00 =
                new LoggedMechanismLigament2d(
                        "targetLigament00",
                        bullseyeLength / 2,
                        0,
                        0.5,
                        DEBUG ? new Color8Bit(Color.kAliceBlue) : BACKROUND_COLOR);
        targetLigament0 =
                new LoggedMechanismLigament2d(
                        "targetLigament0",
                        bullseyeLength,
                        0,
                        0.5,
                        DEBUG ? new Color8Bit(Color.kRed) : new Color8Bit(Color.kRed));

        targetLigament1 =
                new LoggedMechanismLigament2d(
                        "targetLigament1",
                        BULLSEYE_SIZE,
                        0,
                        1,
                        DEBUG ? new Color8Bit(Color.kGreen) : BACKROUND_COLOR);

        targetLigament2 =
                new LoggedMechanismLigament2d(
                        "targetLigament2",
                        bullseyeLength,
                        0,
                        0.5,
                        DEBUG ? new Color8Bit(Color.kCyan) : new Color8Bit(Color.kRed));

        roundXLogger =
                new LoggedMechanismLigament2d("rxl", 0, 0, 3, new Color8Bit(Color.kBlueViolet));
        root.append(targetPosition0);
        targetPosition0.append(targetPosition1);
        targetPosition1.append(targetPosition2);
        targetPosition2.append(targetPosition3);
        targetPosition3.append(targetLigament00);
        targetLigament00.append(targetLigament0);
        targetLigament0.append(targetLigament1);
        targetLigament1.append(targetLigament2);

        root.append(lowerMeasured);
        if (DEBUG) root.append(roundXLogger);

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

    private double pothag(double x, double y) {
        return Math.sqrt((x * x) + (y * y));
    }

    private double halfRound(double num) {
        double ceil = Math.ceil(num * 2) / 2;
        double floor = Math.floor(num * 2) / 2;
        double ceilDiff = Math.abs(ceil - num);
        double floorDiff = Math.abs(num - floor);

        if (floorDiff < ceilDiff) {
            return floor;
        } else {
            return ceil;
        }
    }

    public void setTargetPosition(Translation2d translation) {
        double x = translation.getX();
        double y = translation.getY();
        if (translation.getX() == 0.0 && translation.getY() == 0.0) {
            targetPosition0.setAngle(45);

            targetPosition1.setAngle(-45);
            targetPosition1.setLength(1000);
            update();
            return;
        }

        double targetPosition0Angle = 0.0;
        double targetPosition1Angle = 0.0;
        double targetPosition2Angle = 0.0;
        if (x > 0 && y > 0) {
            targetPosition0Angle = 45;
            targetPosition1Angle = 0;
            targetPosition2Angle = 90;
        } else if (x < 0 && y > 0) {
            targetPosition0Angle = 145;
            targetPosition1Angle = 180;
            targetPosition2Angle = 90;

        } else if (x > 0 && y < 0) {
            targetPosition0Angle = 315;
            targetPosition1Angle = 0;
            targetPosition2Angle = 270;
        } else if (x < 0 && y < 0) {
            targetPosition0Angle = 225;
            targetPosition1Angle = 180;
            targetPosition2Angle = 270;
        }
        targetPosition0.setAngle(targetPosition0Angle);
        targetPosition1.setAngle(targetPosition1Angle - targetPosition0Angle);
        targetPosition2.setAngle(targetPosition2Angle);
        double roundX = halfRound(x);
        double roundY = halfRound(y);

        targetPosition1.setLength(roundX - 0.5);
        targetPosition2.setLength(roundY - 0.5);

        double slope =
                Radians.of(Math.atan2(Math.abs(x - roundX), Math.abs(y - roundY))).in(Degrees);

        targetPosition3.setAngle(slope);
        System.out.println(Boolean.toString(roundX > x) + " " + Boolean.toString(roundY > y));

        targetPosition3.setLength(pothag(Math.abs(y - roundY), Math.abs(x - roundX)));
        if (roundX < x) {
            slope += 45;
        } else {
            slope += 45;
        }

        targetLigament00.setAngle(slope);
        slope += 45;
        targetLigament0.setAngle(slope);
        slope += 45;

        targetLigament1.setAngle(slope);

        targetLigament2.setAngle(slope);
        roundXLogger.setLength(x);

        update();
    }
}
