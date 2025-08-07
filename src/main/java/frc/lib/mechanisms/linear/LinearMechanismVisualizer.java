// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.mechanisms.linear;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.mechanisms.linear.LinearMechanism.LinearMechCharacteristics;

/** Add your docs here. */
public class LinearMechanismVisualizer {

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

    private static final double armLength = 0.25;

    public LinearMechanismVisualizer(String name, LinearMechCharacteristics characteristics)
    {
        this.name = name;
        mechanism = new LoggedMechanism2d(3.0, 3.0, new Color8Bit(Color.kBlack));
        LoggedMechanismRoot2d root = mechanism.getRoot(name + " root", 1.5, 0.0);

        lowerBound =
            new LoggedMechanismLigament2d(name + "lowerBound",
                characteristics.minDistance().in(Meters), 90.0, 3,
                new Color8Bit(Color.kWhite));

        lowerBoundArm = new LoggedMechanismLigament2d(name + "lowerBoundArm", armLength, -90, 3,
            new Color8Bit(Color.kWhite));


        upperBound =
            new LoggedMechanismLigament2d(name + "upperBound",
                characteristics.maxDistance().in(Meters), 90.0, 3,
                new Color8Bit(Color.kWhite));

        upperBoundArm =
            new LoggedMechanismLigament2d(name + "upperBoundArm", armLength, -90.0, 3,
                new Color8Bit(Color.kWhite));

        measured =
            new LoggedMechanismLigament2d(name + "measured",
                characteristics.startingDistance().in(Meters), 90.0,
                3,
                new Color8Bit(Color.kGreen));

        measuredArm =
            new LoggedMechanismLigament2d(name + "measuredArm", armLength, -90, 3,
                new Color8Bit(Color.kGreen));

        trajectory =
            new LoggedMechanismLigament2d(name + "trajectory",
                characteristics.startingDistance().in(Meters), 90.0,
                3,
                new Color8Bit(Color.kYellow));

        trajectoryArm =
            new LoggedMechanismLigament2d(name + "trajectoryArm", armLength, -90, 3,
                new Color8Bit(Color.kYellow));

        goal = new LoggedMechanismLigament2d(name + "goal",
            characteristics.startingDistance().in(Meters),
            90.0, 3,
            new Color8Bit(Color.kRed));

        goalArm =
            new LoggedMechanismLigament2d(name + "goalArm", armLength, -90, 3,
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

    public void setMeasuredDistance(Distance distance)
    {
        measured.setLength(distance.in(Meters));
        SmartDashboard.putData(name + " Visualizer", mechanism);
    }

    public void setTrajectoryDistance(Distance distance)
    {
        trajectory.setLength(distance.in(Meters));
        SmartDashboard.putData(name + " Visualizer", mechanism);
    }

    public void setGoalDistance(Distance distance)
    {
        goal.setLength(distance.in(Meters));
        SmartDashboard.putData(name + " Visualizer", mechanism);
    }
}

