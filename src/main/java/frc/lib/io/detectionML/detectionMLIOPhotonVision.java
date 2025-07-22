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

package frc.lib.io.DetectionML;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.List;

/**
 * A detectionMLIO implementation that uses a camera connected to hardware running the PhotonVision
 * library to detect objects.
 */
public class DetectionMLIOPhotonVision implements DetectionMLIO {
    protected final PhotonCamera camera;
    protected final String ReturnName;
    private final Alert DisconnectedAlert;

    /**
     * Constructs a {@link DetectionMLIOPhotonVision} object with the specified camera name.
     *
     * @param CameraName The name of the camera
     */
    public DetectionMLIOPhotonVision(String CameraName)
    {
        // CameraName is the name of the NetworkTable that PhotonVision is broadcasting information
        // over.
        // The name of the NetworkTable should be the same as the camera’s nickname (from the
        // PhotonVision UI).
        camera = new PhotonCamera(CameraName);
        DisconnectedAlert =
            new Alert("PhotoVision Camera " + CameraName + " is not connected.", AlertType.kError);
        ReturnName = CameraName;
    }

    @Override
    public void updateInputs(DetectionMLIOInputs inputs)
    {
        /* Verify PhotonVision hardware is connected. */
        inputs.connected = camera.isConnected();
        /* Update results. */
        if (inputs.connected) {
            DisconnectedAlert.set(false);
            // PhotonVision container containing all information about stored targets from
            // camera.
            // List retrieved via .getAllUnreadResults() is FIFO, max size 20, and each call clears
            // the queue. Call once per loop().
            List<PhotonPipelineResult> result = camera.getAllUnreadResults();
            boolean HasTargets = !result.isEmpty();
            // Manipulating targets data when HasTargets is false may result in null pointer
            // exception.
            if (HasTargets) {
                // Most recent set of targets.
                List<PhotonTrackedTarget> CurrentTargets = result.get(0).getTargets();
                int TargetSize = CurrentTargets.size();
                // Clear last timestamp's observations to prevent accumulation.
                inputs.LatestTargetObservation.clear();
                // Add all detected targets within most recent pipeline result to
                // inputs.LatestTargetObservation.
                for (int i = 0; i < TargetSize; i++) {
                    inputs.LatestTargetObservation.add(i, new TargetObservation(
                        CurrentTargets.get(i).getDetectedObjectClassID(),
                        CurrentTargets.get(i).getDetectedObjectConfidence(),
                        CurrentTargets.get(i).getArea(),
                        CurrentTargets.get(i).getPitch(),
                        CurrentTargets.get(i).getYaw(),
                        CurrentTargets.get(i).getSkew()));
                }
            } else {
                // Pass
            }
        } else {
            DisconnectedAlert.set(true);
        }
    }

    @Override
    public String getCamera()
    {
        return ReturnName;
    }
}
