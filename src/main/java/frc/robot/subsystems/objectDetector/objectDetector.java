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
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class objectDetector extends SubsystemBase {
    private final DetectionML detectionML;
    private final Drive drive;
    private double range;
    private double heading;
    private double distance;

    public objectDetector(DetectionMLIO io, Drive drive)
    {
        detectionML = new DetectionML(io);
        this.drive = drive;

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
            Logger.recordOutput("Detection/" + "Calculated Range", range);
            Logger.recordOutput("Detection/" + "Calculated Heading", heading);
            Logger.recordOutput("Detection/" + "Calculated Distance", distance);

            Logger.recordOutput("Detection/" + "True Range", 0.0);
            Logger.recordOutput("Detection/" + "True Heading",
                objectDetectorConstants.ALGAE_TARGETS[0].getPose().toPose2d().getTranslation()
                    .minus(drive.getPose().getTranslation()).getAngle().getRadians());
            Logger.recordOutput("Detection/" + "True Distance",
                objectDetectorConstants.ALGAE_TARGETS[0].getPose().toPose2d().getTranslation()
                    .getDistance(drive.getPose().getTranslation()));
        }
    }
}

