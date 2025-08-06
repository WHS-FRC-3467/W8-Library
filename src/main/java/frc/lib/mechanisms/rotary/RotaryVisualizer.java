// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// https://github.com/Mechanical-Advantage/RobotCode2024Public/blob/main/src/main/java/org/littletonrobotics/frc2024/subsystems/superstructure/arm/ArmVisualizer.java

package frc.lib.mechanisms.rotary;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class RotaryVisualizer {

    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismLigament2d measured;
    private final LoggedMechanismLigament2d trajectory;
    private final LoggedMechanismLigament2d goal;
    private final LoggedMechanismLigament2d lowerBound;
    private final LoggedMechanismLigament2d upperBound;
    private final String name;

    public RotaryVisualizer(String name, Distance length, Angle minAngle, Angle maxAngle,
        Angle startingAngle)
    {
        this.name = name;
        mechanism = new LoggedMechanism2d(3.0, 3.0, new Color8Bit(Color.kBlack));
        LoggedMechanismRoot2d root = mechanism.getRoot(name + " root", 1.5, 1.5);

        if (maxAngle.minus(minAngle).in(Rotations) < 1) {
            lowerBound =
                new LoggedMechanismLigament2d(name, length.in(Meters), minAngle.in(Degrees), 3,
                    new Color8Bit(Color.kWhite));

            upperBound =
                new LoggedMechanismLigament2d(name, length.in(Meters), maxAngle.in(Degrees), 3,
                    new Color8Bit(Color.kWhite));
        } else {
            lowerBound =
                new LoggedMechanismLigament2d(name, 0.0, minAngle.in(Degrees), 3,
                    new Color8Bit(Color.kWhite));

            upperBound =
                new LoggedMechanismLigament2d(name, 0.0, maxAngle.in(Degrees), 3,
                    new Color8Bit(Color.kWhite));
        }


        measured =
            new LoggedMechanismLigament2d(name, length.in(Meters), startingAngle.in(Degrees), 3,
                new Color8Bit(Color.kGreen));

        trajectory =
            new LoggedMechanismLigament2d(name, length.in(Meters), startingAngle.in(Degrees), 3,
                new Color8Bit(Color.kYellow));

        goal = new LoggedMechanismLigament2d(name, length.in(Meters), startingAngle.in(Degrees), 3,
            new Color8Bit(Color.kRed));

        root.append(measured);

    }

    public void setAngle(Angle measuredAngle, Angle trajectoryAngle, Angle goalAngle)
    {
        measured.setAngle(Rotation2d.fromRadians(measuredAngle.in(Radians)));
        if (trajectoryAngle != null && goalAngle != null) {
            trajectory.setAngle(Rotation2d.fromRadians(measuredAngle.in(Radians)));
            goal.setAngle(Rotation2d.fromRadians(goalAngle.in(Radians)));
        }

        SmartDashboard.putData(name + " Visualizer", mechanism);
    }
}
