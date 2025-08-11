// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.devices;

import org.littletonrobotics.junction.Logger;
import frc.lib.io.detectionML.DetectionMLIO;
import frc.lib.io.detectionML.DetectionMLIOInputsAutoLogged;
import frc.lib.io.detectionML.DetectionMLIO.TargetObservation;
import java.util.Arrays;
import java.lang.Math;

/**
 * Device level implementation of DetectionML camera. Performs pipeline data operations assuming IO
 * is passed in from upstream (IO layer) and specific hardware is passed in from downstream
 * (subsystem layer).
 */
public class DetectionML {
    // Placeholder for concrete implementation of DetectionMLIO.
    private final DetectionMLIO io;
    // DetectionMLIOInputs (e.g. skew, yaw, objID, etc.) from the AutoLog file.
    private final DetectionMLIOInputsAutoLogged inputs = new DetectionMLIOInputsAutoLogged();

    /*
     * Interface as a data type allows DetectionML to accept various implementations of
     * DetectionMLIO (e.g. DetectionMLIOPhotonVision or DetectionMLIOLimelight).
     */
    public DetectionML(DetectionMLIO io)
    {
        this.io = io;
    }

    /* Periodically retrive most recent DetectionML pipeline results and populate into inputs. */
    public void periodic()
    {
        io.updateInputs(inputs);
        Logger.processInputs(io.getCamera(), inputs);
    }

    /*
     * Returns the latestTargetObservation field (array of TargetObservations [itself a record]) of
     * inputs. Result is an array of object information (e.g. skew, yaw, objID, etc.) from latest
     * pipeline result. Each index contains information for a single detected object.
     */
    public TargetObservation[] getTargetObservations()
    {
        return inputs.latestTargetObservations;
    }

    /*
     * Uses a curve fit of objArea to estimate distance (in.). deltaS = a*objArea^3 + b*objArea^2 +
     * c*objArea + d. Cubic fit required to better match governing physics (tan(x) based). Determine
     * fit coefficients from empirical calibration procedure.
     */
    public double distanceToTarget_SingleFactorArea(TargetObservation targetObservation,
        float a, float b, float c, float d)
    {
        return (a * Math.pow(targetObservation.objArea(), 3)
            + b * Math.pow(targetObservation.objArea(), 2) + c * targetObservation.objArea() + d);
    }

    /**
     * Uses the camera's focal length & trig to estimate range from target; requires no measurement
     * of pitch. Utilizes pinhole model of a camera. Note that camera focal length in pixels = (P *
     * D) / H, where P = perceived width of known object (px), D = known distance from camera (in.),
     * H = known height of object (in.).
     * 
     * @param targetObservation A data type containing vision pipeline results for a single object.
     * @param objectPhysicalHeight_in The physical height of the object being targeted (in.).
     * @param cameraFocalLength_px The camera focal length in pixels as determined by a calibration
     *        procedure.
     * @param cameraCalFactor An empirical calibration factor to account for real lens effects (e.g.
     *        blur, distortion, focus).
     * @return The estimated range to the object (in.).
     */
    public double rangeToTarget_FocalLength(TargetObservation targetObservation,
        double objectPhysicalHeight_in, double cameraFocalLength_px, double cameraCalFactor)
    {
        // Return & sort corners to estimate detected object's digital height in pixels
        double[] objectDigitalCorners_px =
            {targetObservation.cornerOne()[2], targetObservation.cornerTwo()[2],
                    targetObservation.cornerThree()[2], targetObservation.cornerFour()[2]};
        Arrays.sort(objectDigitalCorners_px);
        // Calculate object's digital height
        double objectDigitalHeight_px = objectDigitalCorners_px[objectDigitalCorners_px.length - 1]
            - objectDigitalCorners_px[0];
        // Return estimated range to object (in.)
        return (cameraCalFactor
            * ((objectPhysicalHeight_in * cameraFocalLength_px) / objectDigitalHeight_px));
    }

    /**
     * Estimates range to a target using the target's known elevation. Algorithm similar to
     * {@link org.photonvision.PhotonUtils}. This method can produce more stable results than
     * SolvePNP when well tuned, if the full 6d robot pose is not required. Note that this method
     * requires the camera to have 0 roll (not be skewed clockwise or CCW relative to the floor),
     * and for there to exist a height differential between goal and camera. The larger this
     * differential, the more accurate the distance estimate will be.
     *
     * @param targetObservation A data type containing vision pipeline results for a single target.
     * @param cameraHeight_in The physical height of the camera off the floor in inches.
     * @param targetHeight_in The physical height of the target off the floor in inches. This should
     *        be the height of whatever is being targeted (i.e. if the targeting region is set to
     *        top, this should be the height of the top of the target).
     * @param cameraPitch_deg The pitch of the camera from the horizontal plane in degrees. Positive
     *        values up.
     * @param targetPitch_deg The pitch of the target from the centerline of the camera's lens in
     *        degrees. Positive values up.
     * @param cameraCalFactor An empirical calibration factor to account for real lens effects (e.g.
     *        blur, distortion, focus).
     * @return The estimated range to the target in inches.
     */
    public double rangeToTarget_Pitch(TargetObservation targetObservation,
        double cameraHeight_in,
        double targetHeight_in, double cameraPitch_deg, double cameraCalFactor)
    {
        double tolerance = 1.5; // in.
        // Camera angled down & above target; target above or below camera centerline.
        if (cameraPitch_deg <= 0 && (cameraHeight_in - targetHeight_in) > tolerance) {
            return (cameraCalFactor * ((Math.abs(targetHeight_in - cameraHeight_in))
                / Math.tan(Math.toRadians(Math.abs(cameraPitch_deg + targetObservation.pitch())))));
        } else {
            // To-do: Mathematically verify other camera/target orientation permutations before
            // implementation (e.g. camera up) -- copy & paste may result in sign errors.
            return -1.0d;
        }
    }

    /**
     * Estimates heading to a target using the target's calculated range. Note that this method
     * requires the camera to have 0 roll (not be skewed clockwise or CCW relative to the floor),
     * and for there to exist a finite range between goal and camera. The larger this differential,
     * the more accurate the distance estimate will be.
     *
     * @param targetObservation A data type containing vision pipeline results for a single target.
     * @param cameraYaw_deg The yaw of the camera from the vertical plane in degrees. Positive
     *        values left.
     * @param targetYaw_deg The yaw of the target from the centerline of the camera's lens in
     *        degrees. Positive values left.
     * @param cameraCalFactor An empirical calibration factor to account for real lens effects (e.g.
     *        blur, distortion, focus).
     * @return The estimated heading to the target in inches.
     */
    public double headingToTarget_Yaw(TargetObservation targetObservation, double cameraYaw_deg,
        double targetRange_in, double cameraCalFactor)
    {
        // Camera angled left or right; target left or right of centerline.
        return (cameraCalFactor
            * (Math.tan(Math.toRadians(Math.abs(cameraYaw_deg + targetObservation.yaw())))
                * targetRange_in));
    }

    /**
     * Calculates an overall distance to a target using range, heading, & elevation for use in
     * localization calculations.
     * 
     * @param targetObservation A data type containing vision pipeline results for a single target.
     * @param cameraPitch_deg The pitch of the camera from the horizontal plane in degrees. Positive
     *        values up.
     * @param cameraYaw_deg The yaw of the camera from the vertical plane in degrees. Positive
     *        values left.
     * @param cameraHeight_in The physical height of the camera off the floor in inches.
     * @param targetHeight_in The physical height of the target off the floor in inches. This should
     *        be the height of whatever is being targeted (i.e. if the targeting region is set to
     *        top, this should be the height of the top of the target).
     * @param cameraCalFactor_range An empirical calibration factor to account for real lens effects
     *        (e.g. blur, distortion, focus) -- applies to range calculation only.
     * @param cameraCalFactor_heading An empirical calibration factor to account for real lens
     *        effects (e.g. blur, distortion, focus) -- applies to heading calculation only.
     * @return
     */
    public double translationToTarget(TargetObservation targetObservation, double cameraPitch_deg,
        double cameraYaw_deg, double cameraHeight_in, double targetHeight_in,
        double cameraCalFactor_range,
        double cameraCalFactor_heading)
    {
        // Return range (x-component of displacement vector) (in.)
        double range_in = rangeToTarget_Pitch(targetObservation, cameraHeight_in, targetHeight_in,
            cameraPitch_deg, cameraCalFactor_range);
        // Return heading (y-component of displacement vector) (in.)
        double heading_in = headingToTarget_Yaw(targetObservation, cameraYaw_deg, range_in,
            cameraCalFactor_heading);
        // Calculate z-component of displacement vector (in.)
        double elevation_in = cameraHeight_in - targetHeight_in;
        // to-do: transforms and localization.
        return 1.0d;
        // To-do: Add isConnected method
        // Place to add device level methods
    }
}
