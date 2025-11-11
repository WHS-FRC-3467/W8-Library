/*
 * Copyright (C) 2025 Windham Windup
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

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.devices.AprilTagCamera;

public class VisionSubsystem extends SubsystemBase {
    private AprilTagCamera camera1;
    private AprilTagCamera camera2;

    public VisionSubsystem(AprilTagCamera camera1, AprilTagCamera camera2) {
        this.camera1 = camera1;
        this.camera2 = camera2;
    }

    @Override
    public void periodic() {
        camera1.periodic();
        camera2.periodic();
    }
}
