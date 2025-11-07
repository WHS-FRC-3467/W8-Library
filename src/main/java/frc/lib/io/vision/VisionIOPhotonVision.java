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

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
    private static final Matrix<N3, N3> DEFAULT_CAMERA_INSTRINSICS =
        MatBuilder.fill(Nat.N3(), Nat.N3(),
            2002.948392331919, 0.0, 783.9099067246102,
            0.0, 1999.0390684862123, 662.7694019679813,
            0.0, 0.0, 1.0);

    private static final Vector<N8> DEFAULT_DIST_COEFFS = VecBuilder.fill(
        0.09905119793103302,
        -0.06388083628565337,
        3.87402720846368E-5,
        1.4421218015997156E-4,
        -0.16329892957216433,
        -0.004599206903333014,
        0.0029050841273878885,
        0.0067195798658376375);

    protected final PhotonCamera camera;
    private final Camera cameraProperties;

    private final Alert failedToFetchInstrinsicsAlert;

    /**
     * Creates a new VisionIOPhotonVision.
     *
     * @param name The name of the camera,
     * @param robotToCamera The transform from the robot to the camera
     */
    public VisionIOPhotonVision(String name, Transform3d robotToCamera)
    {
        camera = new PhotonCamera(name);
        failedToFetchInstrinsicsAlert = new Alert(
            "FAILED TO FETCH INTRINSICS FOR CAMERA " + name
                + ". CONTINUING WITH DEFAULT (THRIFTYCAM)",
            AlertType.kError);

        Matrix<N3, N3> cameraIntrinsics = camera.getCameraMatrix().orElseGet(() -> {
            failedToFetchInstrinsicsAlert.set(true);
            return DEFAULT_CAMERA_INSTRINSICS;
        });

        Matrix<N8, N1> distCoeffs = camera.getDistCoeffs().orElseGet(() -> {
            failedToFetchInstrinsicsAlert.set(true);
            return DEFAULT_DIST_COEFFS;
        });

        cameraProperties =
            new Camera(name, robotToCamera,
                cameraIntrinsics,
                distCoeffs);
    }

    private Optional<List<TagObservation>> tagObservationsFromPipelineResult(
        PhotonPipelineResult result)
    {
        if (!result.hasTargets())
            return Optional.empty();

        List<TagObservation> observations = new ArrayList<>();
        result.targets.forEach(target -> {
            var observation = new TagObservation(
                target.fiducialId,
                target.area,
                Degrees.of(target.pitch),
                Degrees.of(target.yaw),
                target.detectedCorners,
                target.bestCameraToTarget);

            observations.add(observation);
        });
        return Optional.of(observations);
    }

    private Optional<VisionObservation> poseObservationFromPipelineResult(
        PhotonPipelineResult result)
    {
        if (!result.hasTargets())
            return Optional.empty();

        Time timestamp = Seconds.of(result.getTimestampSeconds());

        var optionalTagObservations = tagObservationsFromPipelineResult(result);
        if (optionalTagObservations.isEmpty()) {
            return Optional.empty();
        }
        List<TagObservation> tagObservations = optionalTagObservations.get();

        var multiTagResult = result.getMultiTagResult().map(multiTag -> new VisionObservation(
            timestamp,
            cameraProperties,
            multiTag.estimatedPose.best,
            multiTag.estimatedPose.ambiguity,
            tagObservations));

        if (multiTagResult.isPresent()) {
            return Optional.of(multiTagResult.get());
        }

        var bestTarget = result.getBestTarget();
        var singleTagResult = new VisionObservation(
            timestamp,
            cameraProperties,
            bestTarget.bestCameraToTarget,
            bestTarget.poseAmbiguity,
            tagObservations);

        return Optional.of(singleTagResult);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs)
    {
        inputs.connected = camera.isConnected();

        List<VisionObservation> poseObservations = new ArrayList<>();
        camera.getAllUnreadResults().forEach(r -> {
            var observation = poseObservationFromPipelineResult(r);

            if (observation.isEmpty()) {
                return;
            }
            poseObservations.add(observation.get());
        });

        inputs.poseObservations = poseObservations.toArray(new VisionObservation[0]);
    }
}
