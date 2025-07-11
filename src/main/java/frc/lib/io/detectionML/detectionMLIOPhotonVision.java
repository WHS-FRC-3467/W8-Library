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

package frc.lib.io.detectionML;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.List;

/**
 * A detectionMLIO implementation that uses a camera connected to hardware running the PhotonVision
 * library to detect objects.
 */
public class detectionMLIOPhotonVision implements detectionMLIO {
    protected final PhotonCamera camera;
    protected final String returnName;
    private final Alert disconnectedAlert;

    /**
     * Constructs a {@link detectionMLIOPhotonVision} object with the specified camera name.
     *
     * @param cameraName The name of the camera
     */
    public detectionMLIOPhotonVision(String cameraName)
    {
        // cameraName is the name of the NetworkTable that PhotonVision is broadcasting information
        // over.
        // The name of the NetworkTable should be the same as the camera’s nickname (from the
        // PhotonVision UI).
        camera = new PhotonCamera(cameraName);
        disconnectedAlert =
            new Alert("PhotoVision Camera " + cameraName + " is not connected.", AlertType.kError);
        returnName = cameraName;
    }

    @Override
    public void updateInputs(detectionMLIOInputs inputs)
    {
        /* Verify PhotonVision hardware is connected. */
        inputs.connected = camera.isConnected();
        /* Update results. */
        if (inputs.connected) {
            disconnectedAlert.set(false);
            // PhotonVision container containing all information about current targets from
            // "camera".
            // All data within container retrieved via .getLatestResult() are nominally within
            // the same timestamp.
            var result = camera.getLatestResult();
            boolean hasTargets = result.hasTargets();
            // Getting targets when hasTargets is false may result in null pointer exception.
            if (hasTargets) {
                List<PhotonTrackedTarget> targets = result.getTargets();
                // getBestTarget() returns "best" target from sorted list based on "Target Sort"
                // settings. Alternatively, loop through "targets" and return salient information
                // index-wise.
                PhotonTrackedTarget target = result.getBestTarget();
                inputs.objID = target.getDetectedObjectClassID();
                // Object detection confidence (0-1)
                inputs.objConf = target.getDetectedObjectConfidence();
                // Pct of camera feed bounding box occupies (0-100)
                inputs.objArea = target.getArea();
                // Positive up (deg.)
                inputs.pitch = target.getPitch();
                // Positive left (deg.)
                inputs.yaw = target.getYaw();
                // Positive CCW (deg.)
                inputs.skew = target.getSkew();

            } else {
                // Pass
            }
        } else {
            disconnectedAlert.set(true);
        }
    }

    @Override
    public String cameraString()
    {
        return returnName;
    }
}
