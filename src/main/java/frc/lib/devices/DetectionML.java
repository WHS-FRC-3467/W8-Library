// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.devices;

import org.littletonrobotics.junction.Logger;
import frc.lib.io.detectionML.DetectionMLIO;
import frc.lib.io.detectionML.DetectionMLIOInputsAutoLogged;
import frc.lib.io.detectionML.DetectionMLIO.TargetObservation;

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
     * Uses a curve fit of objArea to estimate distance. Does not take object orientation into
     * consideration. e.g. deltaS = a*objArea^2 + b*objArea + c.
     */
    public float distanceToTarget_SingleFactorArea(TargetObservation targetObservation,
        float conversionFactor)
    {
        return 1.0f;
    }

    /*
     * Uses a surface fit of objArea, skew (CW-CCW), pitch (up-down), and yaw (left-right) to
     * estimate distance.
     */
    public float distanceToTarget_MultiFactorArea(TargetObservation targetObservation)
    {
        return 1.0f;
    }

    /* Estimates distance to object as a curve fit of the Y-coordinate of the detObj's centroid. */
    public float distanceToTarget_YCentroid(TargetObservation targetObservation)
    {

        double yCentroid =
            (targetObservation.cornerOne()[1] + targetObservation.cornerTwo()[1]
                + targetObservation.cornerThree()[1] + targetObservation.cornerFour()[1]) / 4.0;
        // TODO: implementation

        return 1.0f;
    }

    /* Uses the camera's FOV, aspect ratio, & trig to estimate distance from target. */
    public float distanceToTarget_FOV(TargetObservation targetObservation)
    {
        return 1.0f;
    }

    // TODO: Add isConnected method
    // Place to add device level methods

}
