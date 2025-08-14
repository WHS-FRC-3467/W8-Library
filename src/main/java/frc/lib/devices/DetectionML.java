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
import edu.wpi.first.math.geometry.Translation3d;

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
    public double rangeToTarget_SingleFactorArea(TargetObservation targetObservation,
        float a, float b, float c, float d)
    {
        return (a * Math.pow(targetObservation.objArea(), 3)
            + b * Math.pow(targetObservation.objArea(), 2) + c * targetObservation.objArea() + d);
    }

    /**
     * Uses the camera's focal length & trig to estimate range from target; requires no measurement
     * of pitch. Utilizes pinhole model of a camera. Note that camera focal length in pixels = (P *
     * D) / H, where P = perceived width of known object (px), D = known distance from camera (in.),
     * H = known height of object (in.). For unreliable corner detection or object digital height
     * calculation, use rangeToTarget_SingleFactorArea.
     * 
     * @param targetObservation A data type containing vision pipeline results for a single object.
     * @param objectPhysicalHeightMeters The physical height of the object being targeted in meters.
     * @param cameraFocalLengthPixels The camera focal length in pixels as determined by a
     *        calibration procedure.
     * @param cameraCalFactor An empirical calibration factor to account for real lens effects (e.g.
     *        blur, distortion, focus).
     * @return The estimated range to the object in meters.
     */
    public double rangeToTarget_FocalLength(TargetObservation targetObservation,
        double objectPhysicalHeightMeters, double cameraFocalLengthPixels, double cameraCalFactor)
    {
        // Return & sort corners to estimate detected object's digital height in pixels
        double[] objectDigitalCorners_px =
            {targetObservation.cornerOne()[2], targetObservation.cornerTwo()[2],
                    targetObservation.cornerThree()[2], targetObservation.cornerFour()[2]};
        Arrays.sort(objectDigitalCorners_px);
        // Calculate object's digital height
        double objectPhysicalHeightPixels =
            objectDigitalCorners_px[objectDigitalCorners_px.length - 1]
                - objectDigitalCorners_px[0];
        // Return estimated range to object (in.)
        return (cameraCalFactor
            * ((objectPhysicalHeightMeters * cameraFocalLengthPixels)
                / objectPhysicalHeightPixels));
    }

    /**
     * Estimates range to a target using the target's known elevation. Algorithm similar to
     * {@link org.photonvision.PhotonUtils}. This method can produce more stable results than
     * SolvePNP when well tuned, if the full 6d robot pose is not required. Note that this method
     * requires the camera to have 0 roll (not be skewed clockwise or CCW relative to the floor),
     * and for there to exist a height differential between goal and camera. The larger this
     * differential, the more accurate the distance estimate will be. For small differentials, use
     * rangeToTarget_FocalLength.
     *
     * @param targetObservation A data type containing vision pipeline results for a single target.
     *        Used to determine the pitch of the target from the centerline of the camera's lens in
     *        degrees. Positive values up.
     * @param cameraHeightMeters The physical height of the camera off the floor in meters.
     * @param targetHeightMeters The physical height of the target off the floor in meters. This
     *        should be the height of whatever is being targeted (i.e. if the targeting region is
     *        set to top, this should be the height of the top of the target).
     * @param cameraPitchDegrees The pitch of the camera from the horizontal plane in degrees.
     *        Positive values up.
     * @param cameraCalFactor An empirical calibration factor to account for real lens effects (e.g.
     *        blur, distortion, focus).
     * @return The estimated range to the target in meters.
     */
    public double rangeToTarget_Pitch(TargetObservation targetObservation,
        double cameraHeightMeters,
        double targetHeightMeters, double cameraPitchDegrees, double cameraCalFactor)
    {
        double tolerance = 1.5 / 39.37; // Empirically-determined tolerance (m)
        // Mathematically verified for camera pitched up or down with target above or below lens
        // centerline.
        if (Math.abs(cameraHeightMeters - targetHeightMeters) > tolerance) {
            return (cameraCalFactor * ((targetHeightMeters - cameraHeightMeters)
                / Math.tan(Math.toRadians(cameraPitchDegrees + targetObservation.pitch()))));
        } else {
            // Use rangeToTarget_FocalLength.
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
     *        Used to determine the yaw of the target from the centerline of the camera's lens in
     *        degrees. Positive values left.
     * @param cameraYawDeg The yaw of the camera from the vertical plane in degrees. Positive values
     *        left.
     * @param targetRangeMeters Range to the target in meters.
     * @param cameraCalFactor An empirical calibration factor to account for real lens effects (e.g.
     *        blur, distortion, focus).
     * @return The estimated heading to the target in meters.
     */
    public double headingToTarget_Yaw(TargetObservation targetObservation, double cameraYawDeg,
        double targetRangeMeters, double cameraCalFactor)
    {
        // Mathematically verified for camera yawed left or right with target left or right of lens
        // centerline. Absolute value required for camera yawed right.
        return (cameraCalFactor
            * (Math.tan(Math.toRadians(Math.abs(cameraYawDeg + targetObservation.yaw())))
                * targetRangeMeters));
    }

    /**
     * Estimates the target's distance from the camera using target's camera-relative range,
     * heading, & elevation.
     *
     * @param cameraYawDeg The yaw of the camera from the vertical plane in degrees. Positive values
     *        left.
     * @param targetRangeMeters Range to the target in meters.
     * @param cameraCalFactor An empirical calibration factor to account for real lens effects (e.g.
     *        blur, distortion, focus).
     * @return The estimated distance from the camera to the target in meters.
     */
    public double headingToTarget_Yaw(double targetRangeMeters, double targetHeadingMeters,
        double targetElevationMeters)
    {
        // Return magnitude of displacement vector between camera and target (m)
        return Math.sqrt((Math.pow(targetRangeMeters, 2) + Math.pow(targetHeadingMeters, 2)
            + Math.pow(targetElevationMeters, 2)));
    }

    /**
     * Calculates the target's camera-relative Translation3d using range, heading, & elevation for
     * use in localization calculations.
     * 
     * @param targetObservation A data type containing vision pipeline results for a single target.
     * @param cameraPitchDegrees The pitch of the camera from the horizontal plane in degrees.
     *        Positive values up.
     * @param cameraYawDeg The yaw of the camera from the vertical plane in degrees. Positive values
     *        left.
     * @param cameraHeightMeters The physical height of the camera off the floor in meters.
     * @param targetHeightMeters The physical height of the target off the floor in meters. This
     *        should be the height of whatever is being targeted (i.e. if the targeting region is
     *        set to top, this should be the height of the top of the target).
     * @param cameraCalFactorRange An empirical calibration factor to account for real lens effects
     *        (e.g. blur, distortion, focus) -- applies to range calculation only.
     * @param cameraCalFactorHeading An empirical calibration factor to account for real lens
     *        effects (e.g. blur, distortion, focus) -- applies to heading calculation only.
     * @return The target's camera-relative 3D translataion.
     */
    public Translation3d translationToTarget_3D(TargetObservation targetObservation,
        double cameraPitchDegrees,
        double cameraYawDeg, double cameraHeightMeters, double targetHeightMeters,
        double cameraCalFactorRange,
        double cameraCalFactorHeading)
    {
        // Calculate range (x-component of target's camera-relative displacement vector) (m)
        double rangeMeters =
            rangeToTarget_Pitch(targetObservation, cameraHeightMeters, targetHeightMeters,
                cameraPitchDegrees, cameraCalFactorRange);
        // Calculate heading (y-component of target's camera-relative position vector) (m)
        double headingMeters = headingToTarget_Yaw(targetObservation, cameraYawDeg, rangeMeters,
            cameraCalFactorHeading);
        // Calculate elevation (z-component of target's camera-relative position vector) (m)
        double elevationMeters = cameraHeightMeters - targetHeightMeters;
        // Return 3D Translation to target
        return new Translation3d(rangeMeters, headingMeters, elevationMeters);
    }

    // To-do: Add isConnected method
    // Place to add device level methods

}
