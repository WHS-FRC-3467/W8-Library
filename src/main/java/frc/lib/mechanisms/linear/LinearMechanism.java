// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.mechanisms.linear;

import edu.wpi.first.units.measure.Distance;
import frc.lib.mechanisms.Mechanism;
import frc.lib.util.MechanismUtil.DistanceAngleConverter;

/** Add your docs here. */
public interface LinearMechanism extends Mechanism {

    public record LinearMechCharacteristics(
        Distance minDistance,
        Distance maxDistance,
        Distance startingDistance,
        DistanceAngleConverter converter) {
    }
}
