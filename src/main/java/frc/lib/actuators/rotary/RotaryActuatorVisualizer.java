// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// https://github.com/Mechanical-Advantage/RobotCode2024Public/blob/main/src/main/java/org/littletonrobotics/frc2024/subsystems/superstructure/arm/ArmVisualizer.java

package frc.lib.actuators.rotary;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Rotations;

import frc.lib.actuators.rotary.RotaryActuator.RotaryMechCharacteristics;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;

import java.util.Optional;

/**
 * A visualizer for rotary mechanisms that displays the current angle, trajectory, and goal angle
 * using a LoggedMechanism2d.
 */
public class RotaryActuatorVisualizer {

    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismLigament2d measured;
    private final LoggedMechanismLigament2d trajectory;
    private final LoggedMechanismLigament2d goal;
    private final LoggedMechanismLigament2d lowerBound;
    private final LoggedMechanismLigament2d upperBound;
    private final String name;

    private final double armLength;

    public RotaryActuatorVisualizer(String name, RotaryMechCharacteristics constants) {
        this.name = name;
        mechanism = new LoggedMechanism2d(3.0, 3.0, new Color8Bit(Color.BLACK));
        LoggedMechanismRoot2d root = mechanism.getRoot(name + " root", 1.5, 1.5);

        armLength = constants.armLength().in(Meters);

        if (constants.maxAngle().minus(constants.minAngle()).in(Rotations) < 1) {
            lowerBound =
                    new LoggedMechanismLigament2d(
                            name + "Lower Bound",
                            constants.armLength().in(Meters),
                            constants.minAngle().in(Degrees),
                            3,
                            new Color8Bit(Color.WHITE));

            upperBound =
                    new LoggedMechanismLigament2d(
                            name + "Upper Bound",
                            constants.armLength().in(Meters),
                            constants.maxAngle().in(Degrees),
                            3,
                            new Color8Bit(Color.WHITE));
        } else {
            lowerBound =
                    new LoggedMechanismLigament2d(
                            name + "Lower Bound",
                            0.0,
                            constants.minAngle().in(Degrees),
                            3,
                            new Color8Bit(Color.WHITE));

            upperBound =
                    new LoggedMechanismLigament2d(
                            name + "Upper Bound",
                            0.0,
                            constants.maxAngle().in(Degrees),
                            3,
                            new Color8Bit(Color.WHITE));
        }

        measured =
                new LoggedMechanismLigament2d(
                        name + "Measured",
                        armLength,
                        constants.startingAngle().in(Degrees),
                        3,
                        new Color8Bit(Color.GREEN));

        trajectory =
                new LoggedMechanismLigament2d(
                        name + "Trajectory",
                        armLength,
                        constants.startingAngle().in(Degrees),
                        3,
                        new Color8Bit(Color.YELLOW));

        goal =
                new LoggedMechanismLigament2d(
                        name + "Goal",
                        armLength,
                        constants.startingAngle().in(Degrees),
                        3,
                        new Color8Bit(Color.RED));

        root.append(lowerBound);
        root.append(upperBound);
        root.append(measured);
        root.append(trajectory);
        root.append(goal);
    }

    private void update() {
        Logger.recordOutput("Mechanism Visualizers/" + name + " Visualizer", mechanism);
    }

    /**
     * Sets the current measured angle of the rotary mechanism.
     *
     * @param angle The measured angle to display
     */
    public void setCurrentAngle(Angle angle) {
        measured.setAngle(Rotation2d.fromRadians(angle.in(Radians)));

        update();
    }

    /**
     * Sets the trajectory angle setpoint for the rotary mechanism.
     *
     * @param angle Optional trajectory angle, empty to hide
     */
    public void setTrajectoryAngle(Optional<Angle> angle) {
        if (angle.isEmpty()) {
            trajectory.setLength(0.0);
        }

        angle.ifPresent(
                (a) -> {
                    trajectory.setLength(armLength);
                    trajectory.setAngle(a.in(Degrees));
                });

        update();
    }

    /**
     * Sets the goal angle for the rotary mechanism.
     *
     * @param angle Optional goal angle, empty to hide
     */
    public void setGoalAngle(Optional<Angle> angle) {
        if (angle.isEmpty()) {
            goal.setLength(0.0);
        }

        angle.ifPresent(
                (a) -> {
                    goal.setLength(armLength);
                    goal.setAngle(a.in(Degrees));
                });

        update();
    }
}
