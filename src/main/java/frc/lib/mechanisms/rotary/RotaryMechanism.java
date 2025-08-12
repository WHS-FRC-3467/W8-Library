// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.mechanisms.rotary;

import frc.lib.mechanisms.Mechanism;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public interface RotaryMechanism extends Mechanism {

    public static record RotaryMechCharacteristics(
        Distance armLength,
        Angle minAngle,
        Angle maxAngle,
        Angle startingAngle) {
    }

}
