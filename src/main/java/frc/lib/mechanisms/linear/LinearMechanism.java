// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.mechanisms.linear;

import edu.wpi.first.units.measure.Distance;
import frc.lib.mechanisms.Mechanism;
import frc.lib.util.MechanismUtil.DistanceAngleConverter;

/**
 * Interface for linear mechanisms, which are mechanisms that move in a straight line. This
 * interface extends the Mechanism interface and provides characteristics specific to linear
 * mechanisms.
 */
public interface LinearMechanism extends Mechanism {

    public record LinearMechCharacteristics(
        Distance minDistance,
        Distance maxDistance,
        Distance startingDistance,
        DistanceAngleConverter converter) {
    }
}
