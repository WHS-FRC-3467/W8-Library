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

package frc.lib.io.objectdetection;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.List;

/**
 * An ObjectDetectionIO implementation that uses a camera connected to hardware running the
 * PhotonVision library to detect objects.
 */
public class ObjectDetectionIOPhotonVision implements ObjectDetectionIO {
    protected final PhotonCamera camera;
    protected final String cameraName;
    private final Alert disconnectedAlert;

    /**
     * Constructs a {@link ObjectDetectionIOPhotonVision} object with the specified camera name.
     *
     * @param cameraName The name of the camera
     */
    public ObjectDetectionIOPhotonVision(String cameraName)
    {
        // CameraName is the name of the NetworkTable that PhotonVision is broadcasting information
        // over.
        // The name of the NetworkTable should be the same as the camera’s nickname (from the
        // PhotonVision UI).
        camera = new PhotonCamera(cameraName);
        disconnectedAlert =
            new Alert("PhotoVision Camera " + cameraName + " is not connected.", AlertType.kError);
        this.cameraName = cameraName;
    }

    @Override
    public void updateInputs(ObjectDetectionIOInputs inputs) {
        /* Verify PhotonVision hardware is connected. */
        inputs.connected = camera.isConnected();
        if (!inputs.connected) {
            disconnectedAlert.set(true);
            return;
        }
        /* Update results. */
        disconnectedAlert.set(false);
        // PhotonVision container containing all information about stored targets from
        // camera.
        // List retrieved via .getAllUnreadResults() is FIFO, max size 20, and each call clears
        // the queue. Call once per loop().
        List<PhotonPipelineResult> result = camera.getAllUnreadResults();
        // Manipulating targets data when result is empty may result in null pointer
        // exception.
        if (result.isEmpty()) {
            return;
        }
        // Most recent set of targets.
        List<PhotonTrackedTarget> currentTargets = result.get(0).getTargets();
        int targetSize = currentTargets.size();
        // Clear last timestamp's observations to prevent accumulation (by re-creating
        // array, similar to .clear()).
        inputs.latestTargetObservations = new TargetObservation[targetSize];
        // Add all detected targets within most recent pipeline result to
        // inputs.LatestTargetObservation.
        for (int i = 0; i < targetSize; i++) {
            var corner1 = List.of(currentTargets.get(i).getMinAreaRectCorners().get(0).x,
                currentTargets.get(i).getMinAreaRectCorners().get(0).y);
            var corner2 = List.of(currentTargets.get(i).getMinAreaRectCorners().get(1).x,
                currentTargets.get(i).getMinAreaRectCorners().get(1).y);
            var corner3 = List.of(currentTargets.get(i).getMinAreaRectCorners().get(2).x,
                currentTargets.get(i).getMinAreaRectCorners().get(2).y);
            var corner4 = List.of(currentTargets.get(i).getMinAreaRectCorners().get(3).x,
                currentTargets.get(i).getMinAreaRectCorners().get(3).y);
            inputs.latestTargetObservations[i] = new TargetObservation(
                currentTargets.get(i).getDetectedObjectClassID(),
                currentTargets.get(i).getDetectedObjectConfidence(),
                currentTargets.get(i).getArea(),
                currentTargets.get(i).getPitch(),
                currentTargets.get(i).getYaw(),
                currentTargets.get(i).getSkew(),
                corner1,
                corner2,
                corner3,
                corner4); // Corners: origin top-left, x positive right, y positive down.
        }

    }

    @Override
    public String getCamera() {
        return cameraName;
    }
}
