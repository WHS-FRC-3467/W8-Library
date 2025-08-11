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

    /*
     * Uses the camera's focal length & trig to estimate distance (in.) from target. Utilizes
     * pinhole model of a camera; calibration factor is purely empirical and accounts for physical
     * lens effects like blur, edge distortion, focus, etc. Camera focal length in pixels (from
     * calibration) = (P * D) / H, where P - perceived width of known object (px), D - known
     * distance from camera (in.), H - known height of object (in.)
     */
    public double distanceToTarget_FocalLength(TargetObservation targetObservation,
        double objectPhysicalHeight_in, double cameraFocalLength_px, double cameraCalFactor)
    {
        // Return & sort corners to estimate detected object's digital height in pixels
        double[] objectDigitalCorners_px =
            {targetObservation.cornerOne()[2], targetObservation.cornerTwo()[2],
                    targetObservation.cornerThree()[2], targetObservation.cornerFour()[2]};
        Arrays.sort(objectDigitalCorners_px);
        // Calculate digital height
        double objectDigitalHeight_px = objectDigitalCorners_px[objectDigitalCorners_px.length - 1]
            - objectDigitalCorners_px[0];
        // Return estimated distance to object (in.)
        return (cameraCalFactor
            * ((objectPhysicalHeight_in * cameraFocalLength_px) / objectDigitalHeight_px));
    }

    /*
     * Calculates distance (in.) to target using object's detected pitch in relation to the camera's
     * pitch from the floor; calibration factor is purely empirical and accounts for physical lens
     * effects like blur, edge distortion, focus, etc. Note that pitch is positive where object is
     * above camera C.L. and negative where below. Small height differentials & camera roll may
     * introduce instabilities.
     */
    public double distanceToTarget_Skew(TargetObservation targetObservation, double cameraHeight_in,
        double targetHeight_in, double cameraPitch_deg, double cameraCalFactor)
    {
        double tolerance = 1.5; // in.
        // Camera neutral or angled down & above target; target can be either above or below camera
        // centerline.
        if (cameraPitch_deg <= 0 && (cameraHeight_in - targetHeight_in) > tolerance) {
            return (cameraCalFactor * ((Math.abs(targetHeight_in - cameraHeight_in))
                / Math.tan(Math.toRadians(Math.abs(cameraPitch_deg + targetObservation.pitch())))));
        } else {
            return -1.0f;
            // To-do: other camera/target permutations.
        }
    }

    // To-do: Add isConnected method
    // Place to add device level methods

}
