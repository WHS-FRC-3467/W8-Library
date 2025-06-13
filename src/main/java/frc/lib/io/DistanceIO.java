// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.io;

import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public abstract class DistanceIO {
    private final String name;

    public DistanceIO(String name) {
        this.name = name;
    }

    public abstract Distance getDistance();

    public boolean isNearDistance(Distance expected, Distance tolerDistance) {
        return MathUtil.isNear(expected.in(Millimeters),getDistance().in(Millimeters),tolerDistance.in(Millimeters));
    }

}
