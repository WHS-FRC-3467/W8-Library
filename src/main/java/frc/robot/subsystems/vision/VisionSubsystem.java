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
import frc.lib.devices.apriltagcameras.AprilTagCameraPhotonVision;

public class VisionSubsystem extends SubsystemBase {
    private AprilTagCameraPhotonVision camera1;
    private AprilTagCameraPhotonVision camera2;

    public VisionSubsystem(AprilTagCameraPhotonVision camera1, AprilTagCameraPhotonVision camera2)
    {
        this.camera1 = camera1;
        this.camera2 = camera2;
    }

    @Override
    public void periodic()
    {
        camera1.periodic();
        camera2.periodic();
    }
}
