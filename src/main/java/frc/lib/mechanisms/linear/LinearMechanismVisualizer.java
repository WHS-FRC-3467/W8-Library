// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.mechanisms.linear;

import static edu.wpi.first.units.Units.Meters;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.mechanisms.linear.LinearMechanism.LinearMechCharacteristics;

/**
 * A visualizer for linear mechanisms that displays the current distance, trajectory, and goal
 * distance using a LoggedMechanism2d.
 */
public class LinearMechanismVisualizer {

    private static final double ARM_LENGTH = 0.25;

    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismLigament2d measured;
    private final LoggedMechanismLigament2d measuredArm;
    private final LoggedMechanismLigament2d trajectory;
    private final LoggedMechanismLigament2d trajectoryArm;
    private final LoggedMechanismLigament2d goal;
    private final LoggedMechanismLigament2d goalArm;
    private final LoggedMechanismLigament2d lowerBound;
    private final LoggedMechanismLigament2d lowerBoundArm;
    private final LoggedMechanismLigament2d upperBound;
    private final LoggedMechanismLigament2d upperBoundArm;
    private final String name;

    private final Pose3d offset;

    public LinearMechanismVisualizer(String name, LinearMechCharacteristics characteristics)
    {
        this.name = name;
        mechanism = new LoggedMechanism2d(3.0, 3.0, new Color8Bit(Color.kBlack));
        LoggedMechanismRoot2d root = mechanism.getRoot(name + " root", 1.5, 0.0);

        offset = new Pose3d(characteristics.offset(), Rotation3d.kZero);

        lowerBound =
            new LoggedMechanismLigament2d(name + "lowerBound",
                characteristics.minDistance().in(Meters), 90.0, 3,
                new Color8Bit(Color.kWhite));

        lowerBoundArm = new LoggedMechanismLigament2d(name + "lowerBoundArm", ARM_LENGTH, -90, 3,
            new Color8Bit(Color.kWhite));


        upperBound =
            new LoggedMechanismLigament2d(name + "upperBound",
                characteristics.maxDistance().in(Meters), 90.0, 3,
                new Color8Bit(Color.kWhite));

        upperBoundArm =
            new LoggedMechanismLigament2d(name + "upperBoundArm", ARM_LENGTH, -90.0, 3,
                new Color8Bit(Color.kWhite));

        measured =
            new LoggedMechanismLigament2d(name + "measured",
                characteristics.startingDistance().in(Meters), 90.0,
                3,
                new Color8Bit(Color.kGreen));

        measuredArm =
            new LoggedMechanismLigament2d(name + "measuredArm", ARM_LENGTH, -90, 3,
                new Color8Bit(Color.kGreen));

        trajectory =
            new LoggedMechanismLigament2d(name + "trajectory",
                characteristics.startingDistance().in(Meters), 90.0,
                3,
                new Color8Bit(Color.kYellow));

        trajectoryArm =
            new LoggedMechanismLigament2d(name + "trajectoryArm", ARM_LENGTH, -90, 3,
                new Color8Bit(Color.kYellow));

        goal = new LoggedMechanismLigament2d(name + "goal",
            characteristics.startingDistance().in(Meters),
            90.0, 3,
            new Color8Bit(Color.kRed));

        goalArm =
            new LoggedMechanismLigament2d(name + "goalArm", ARM_LENGTH, -90, 3,
                new Color8Bit(Color.kRed));

        root.append(lowerBound);
        lowerBound.append(lowerBoundArm);
        root.append(upperBound);
        upperBound.append(upperBoundArm);
        root.append(measured);
        measured.append(measuredArm);
        root.append(trajectory);
        trajectory.append(trajectoryArm);
        root.append(goal);
        goal.append(goalArm);
    }

    private void update()
    {
        SmartDashboard.putData(name + " Visualizer", mechanism);
        Logger.recordOutput(name + "/Pose3d",
            offset.plus(new Transform3d(measured.getLength(), 0, 0,
                Rotation3d.kZero)));
    }

    public void setMeasuredDistance(Distance distance)
    {
        measured.setLength(distance.in(Meters));

        update();
    }

    public void setTrajectoryDistance(Optional<Distance> distance)
    {
        if (distance.isEmpty()) {
            trajectoryArm.setLength(0.0);
        }

        distance.ifPresent(d -> {
            trajectoryArm.setLength(ARM_LENGTH);
            trajectory.setLength(d.in(Meters));
        });

        update();
    }

    public void setGoalDistance(Optional<Distance> distance)
    {
        if (distance.isEmpty()) {
            goalArm.setLength(0.0);
        }

        distance.ifPresent(d -> {
            goalArm.setLength(ARM_LENGTH);
            goal.setLength(d.in(Meters));
        });

        update();
    }
}
