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

package frc.lib.io.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
    protected final PhotonCamera camera;

    /**
     * Creates a new VisionIOPhotonVision.
     *
     * @param name The configured name of the camera.
     */
    public VisionIOPhotonVision(String name)
    {
        camera = new PhotonCamera(name);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs)
    {
        inputs.connected = camera.isConnected();
        final var cameraMatrix = camera.getCameraMatrix();
        if (inputs.cameraMatrix.isEmpty() && cameraMatrix.isPresent()) {
            inputs.cameraMatrix = cameraMatrix;
        }
        final var distCoeffs = camera.getDistCoeffs();
        if (inputs.distCoeffs.isEmpty() && distCoeffs.isPresent()) {
            inputs.distCoeffs = distCoeffs;
        }

        inputs.results = camera.getAllUnreadResults().toArray(new PhotonPipelineResult[0]);
    }
}
