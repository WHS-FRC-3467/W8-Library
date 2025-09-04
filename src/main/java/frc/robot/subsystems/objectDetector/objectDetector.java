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

package frc.robot.subsystems.objectDetector;

import frc.lib.devices.DetectionML;
import frc.lib.io.detectionML.DetectionMLIO;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class objectDetector extends SubsystemBase {
    private final DetectionML detectionML;
    private double range;
    private double heading;
    private double distance;

    public objectDetector(DetectionMLIO io)
    {
        detectionML = new DetectionML(io);

    }

    @Override
    public void periodic()
    {
        detectionML.periodic();

        if (detectionML.getTargetObservations().length > 0) {
            range = detectionML.rangeToTarget_Pitch(detectionML.getTargetObservations()[0], 1, .5,
                0, 1, 0);
            heading = detectionML.headingToTarget_Yaw(detectionML.getTargetObservations()[0], 0,
                range, 1, 0);
            distance = detectionML.distanceToTarget2d(range, heading);
            Logger.recordOutput("Detection/" + "Range", range);
            Logger.recordOutput("Detection/" + "Heading", heading);
            Logger.recordOutput("Detection/" + "Distance", distance);
        }
    }
}

