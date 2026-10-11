// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.mechanisms.flywheel;

import static org.wpilib.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;

/**
 * The {@code FlywheelVisualizer} class is responsible for visualizing a flywheel mechanism using
 * the Mechanism2d library. It creates a graphical representation of the flywheel and its
 * components, which can be displayed on the SmartDashboard.
 *
 * <p>This class initializes a {@code LoggedMechanism2d} object with specified dimensions and
 * constructs the flywheel mechanism using {@code LoggedMechanismLigament2d} objects. The flywheel
 * is represented as a series of connected ligaments forming a circular structure.
 *
 * <p>Example Usage:
 *
 * <pre>
 * FlywheelVisualizer visualizer = new FlywheelVisualizer("Flywheel");
 * </pre>
 */
public class FlywheelActuatorVisualizer {

    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismLigament2d roller;
    private final String name;

    private static double HEIGHT = .50; // Controls the height of the mech2d SmartDashboard
    private static double WIDTH = .50; // Controls width of the mech2d SmartDashboard

    public FlywheelActuatorVisualizer(String name) {
        this.name = name;
        mechanism = new LoggedMechanism2d(WIDTH, HEIGHT);

        roller =
                mechanism
                        .getRoot("pivotPoint_", WIDTH / 2.0, HEIGHT / 2.0)
                        .append(
                                new LoggedMechanismLigament2d(
                                        "roller ", .2, 0, 0, new Color8Bit(Color.ALICE_BLUE)));

        LoggedMechanismLigament2d side1 =
                roller.append(
                        new LoggedMechanismLigament2d(
                                "side1 ", 0.15307, 112.5, 6, new Color8Bit(Color.ALICE_BLUE)));
        LoggedMechanismLigament2d side2 =
                side1.append(
                        new LoggedMechanismLigament2d(
                                "side2 ", 0.15307, 45, 6, new Color8Bit(Color.ALICE_BLUE)));
        LoggedMechanismLigament2d side3 =
                side2.append(
                        new LoggedMechanismLigament2d(
                                "side3 ", 0.15307, 45, 6, new Color8Bit(Color.ALICE_BLUE)));
        LoggedMechanismLigament2d side4 =
                side3.append(
                        new LoggedMechanismLigament2d(
                                "side4 ", 0.15307, 45, 6, new Color8Bit(Color.ALICE_BLUE)));
        LoggedMechanismLigament2d side5 =
                side4.append(
                        new LoggedMechanismLigament2d(
                                "side5 ", 0.15307, 45, 6, new Color8Bit(Color.ALICE_BLUE)));
        LoggedMechanismLigament2d side6 =
                side5.append(
                        new LoggedMechanismLigament2d(
                                "side6 ", 0.15307, 45, 6, new Color8Bit(Color.ALICE_BLUE)));
        LoggedMechanismLigament2d side7 =
                side6.append(
                        new LoggedMechanismLigament2d(
                                "side7 ", 0.15307, 45, 6, new Color8Bit(Color.ALICE_BLUE)));
        side7.append(
                new LoggedMechanismLigament2d(
                        "side8 ", 0.15307, 45, 6, new Color8Bit(Color.ALICE_BLUE)));
    }

    /**
     * Sets the rotation angle of the flywheel visualization.
     *
     * @param angle The angle to display
     */
    public void setAngle(Angle angle) {
        roller.setAngle(angle.in(Degrees));
        update();
    }

    /**
     * Sets the background color of the visualization.
     *
     * @param color The background color to use
     */
    public void setColor(Color color) {
        mechanism.setBackgroundColor(new Color8Bit(color));
    }

    private void update() {
        Logger.recordOutput("Mechanism Visualizers/" + name + " Visualizer", mechanism);
    }
}
