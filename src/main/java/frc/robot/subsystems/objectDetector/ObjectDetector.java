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

import frc.lib.devices.ObjectDetection;
import frc.lib.io.objectdetection.ObjectDetectionIO;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;

public class ObjectDetector extends SubsystemBase {
    private final ObjectDetection objectDetection;
    private final Drive drive;
    private double range, heading, distance;
    private Translation2d targetLocation;
    private ArrayList<Translation2d> lastNDetections = new ArrayList<>(10);

    // Pass in any object detection IO implementation (e.g. PhotonVision) that implements
    // objectDetectionIO interface [real or sim]
    public ObjectDetector(ObjectDetectionIO io, Drive drive)
    {
        objectDetection = new ObjectDetection(io);
        this.drive = drive;

    }

    @Override
    public void periodic()
    {
        objectDetection.periodic();

        if (objectDetection.getTargetObservations().length > 0) {
            range = objectDetection.rangeToTarget_Pitch(objectDetection.getTargetObservations()[0],
                ObjectDetectorConstants.CAMERA0_TRANSFORM,
                ObjectDetectorConstants.algaeHeightMeters / 2,
                1, 0);
            heading =
                objectDetection.headingToTarget_Yaw(objectDetection.getTargetObservations()[0],
                    ObjectDetectorConstants.CAMERA0_TRANSFORM,
                    range, 1, 0);
            distance = objectDetection.distanceToTarget2d(range, heading);
            targetLocation =
                objectDetection.estimateTargetToField(range, heading, drive.getPose());
            objectDetection.getLastNDetections(10, lastNDetections, 0.4572,
                targetLocation);

            Logger.recordOutput("Detection/" + "Calculated Range", range);
            Logger.recordOutput("Detection/" + "Calculated Heading", heading);
            Logger.recordOutput("Detection/" + "Calculated Distance", distance);
            Logger.recordOutput("Detection/" + "Latest Detection's Calculated Coordinates",
                targetLocation);
            Logger.recordOutput("Detection/" + "Newest Detection",
                lastNDetections.get(lastNDetections.size() - 1));
            Logger.recordOutput("Detection/" + "Oldest Detection", lastNDetections.get(0));
            Logger.recordOutput("Detection/" + "Detection List Size", lastNDetections.size());

            Logger.recordOutput("Detection/" + "Sim Target #0 True Range",
                ObjectDetectorConstants.SIM_TARGETS[0].getPose().toPose2d().getTranslation()
                    .minus(drive.getPose().getTranslation()).getX());
            Logger.recordOutput("Detection/" + "Sim Target #0 True Heading",
                ObjectDetectorConstants.SIM_TARGETS[0].getPose().toPose2d().getTranslation()
                    .minus(drive.getPose().getTranslation()).getY());
            Logger.recordOutput("Detection/" + "Sim Target #0 True Distance",
                ObjectDetectorConstants.SIM_TARGETS[0].getPose().toPose2d().getTranslation()
                    .getDistance(drive.getPose().getTranslation()));
        }
    }
}
