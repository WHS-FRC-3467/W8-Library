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
package frc.lib.mechanisms.doublejointedarm;

import static edu.wpi.first.units.Units.Radians;

import frc.lib.io.absoluteencoder.AbsoluteEncoderIOSim;
import frc.lib.io.motor.MotorIOSim;

public class DoubleJointedArmMechanismSim
        extends DoubleJointedArmMechanism<
                MotorIOSim, AbsoluteEncoderIOSim, MotorIOSim, AbsoluteEncoderIOSim> {
    public DoubleJointedArmMechanismSim(
            ArmJointMechanismSim upperArm, ArmJointMechanismSim lowerArm, String name) {
        super(upperArm, lowerArm, name);
    }

    @Override
    public void periodic() {
        super.periodic();
        ArmJointMechanismSim.class
                .cast(this.getLowerArm())
                .updateSimAttachedVelocity(this.getUpperArm().getVelocity());

        ArmJointMechanismSim.class
                .cast(this.getUpperArm())
                .updateSimAttachedAngle(this.getLowerArm().getPosition().in(Radians));
    }
}
