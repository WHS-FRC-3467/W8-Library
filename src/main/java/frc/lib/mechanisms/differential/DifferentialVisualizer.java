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

package frc.lib.mechanisms.differential;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import frc.lib.mechanisms.differential.DifferentialMechanism.DiffMechCharacteristics;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * Visualizes a differential mechanism using two ligaments on a {@link LoggedMechanism2d}:
 *
 * <ul>
 *   <li>A <b>green</b> ligament represents the <em>average axis</em> (both motors together).
 *   <li>A <b>blue</b> ligament represents the <em>differential axis</em> (the twist).
 * </ul>
 *
 * <p>The visualization is published to SmartDashboard / AdvantageScope under {@code "Mechanism
 * Visualizers/<name> Visualizer"}.
 */
public class DifferentialVisualizer {

    private static final double CANVAS_SIZE = 3.0;
    private static final double ARM_LENGTH = 1.0;

    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismLigament2d averageLigament;
    private final LoggedMechanismLigament2d differentialLigament;
    private final String name;

    /**
     * Creates a new visualizer for the given differential mechanism.
     *
     * @param name the mechanism name (used as the SmartDashboard key)
     * @param characteristics the mechanism characteristics (starting positions used for initial
     *     angles)
     */
    public DifferentialVisualizer(String name, DiffMechCharacteristics characteristics) {
        this.name = name;

        mechanism = new LoggedMechanism2d(CANVAS_SIZE, CANVAS_SIZE, new Color8Bit(Color.kBlack));

        // Average-axis root: left side of the canvas
        LoggedMechanismRoot2d avgRoot =
                mechanism.getRoot(name + " Average Root", CANVAS_SIZE * 0.25, CANVAS_SIZE * 0.5);

        // Differential-axis root: right side of the canvas
        LoggedMechanismRoot2d diffRoot =
                mechanism.getRoot(
                        name + " Differential Root", CANVAS_SIZE * 0.75, CANVAS_SIZE * 0.5);

        // Green ligament for the average axis (both motors together)
        averageLigament =
                new LoggedMechanismLigament2d(
                        name + " Average",
                        ARM_LENGTH,
                        characteristics.startingAverage().in(Degrees),
                        4,
                        new Color8Bit(Color.kGreen));

        // Blue ligament for the differential axis (the twist / difference)
        differentialLigament =
                new LoggedMechanismLigament2d(
                        name + " Differential",
                        ARM_LENGTH,
                        characteristics.startingDifference().in(Degrees),
                        4,
                        new Color8Bit(Color.kCyan));

        avgRoot.append(averageLigament);
        diffRoot.append(differentialLigament);
    }

    /**
     * Updates the average-axis ligament angle.
     *
     * @param angle the current average-axis position
     */
    public void setAverageAngle(Angle angle) {
        averageLigament.setAngle(angle.in(Degrees));
        update();
    }

    /**
     * Updates the differential-axis ligament angle.
     *
     * @param angle the current differential-axis position
     */
    public void setDifferentialAngle(Angle angle) {
        differentialLigament.setAngle(angle.in(Degrees));
        update();
    }

    /** Publishes the current mechanism state to SmartDashboard. */
    private void update() {
        SmartDashboard.putData("Mechanism Visualizers/" + name + " Visualizer", mechanism);
    }
}
