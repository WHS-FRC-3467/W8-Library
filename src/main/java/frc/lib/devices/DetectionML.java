// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.devices;

import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;
import frc.lib.io.detectionML.DetectionMLIO;
import frc.lib.io.detectionML.DetectionMLIOInputsAutoLogged;
import frc.lib.io.detectionML.DetectionMLIO.TargetObservation;
import java.util.Arrays;
import java.lang.Math;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

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

    /**
     * Uses an empirical curve fit of objArea to estimate distance (in.). deltaS = a*objArea^3 +
     * b*objArea^2 + c*objArea + d. Cubic fit required to better match governing physics (tan(x)
     * based). Determine fit coefficients from calibration procedure.
     * 
     * @param targetObservation A data type containing vision pipeline results for a single object.
     * @param a Coefficient for cubic term in curve fit equation.
     * @param b Coefficient for quadratic term in curve fit equation.
     * @param c Coefficient for linear term in curve fit equation.
     * @param d Coefficient for constant term in curve fit equation.
     * @return The estimated range to the object in meters.
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
        // Return & sort corners to estimate detected object's digital height in pixels.
        double[] objectDigitalCorners_px =
            {targetObservation.cornerOne()[2], targetObservation.cornerTwo()[2],
                    targetObservation.cornerThree()[2], targetObservation.cornerFour()[2]};
        Arrays.sort(objectDigitalCorners_px);
        // Calculate object's digital height in pixels.
        double objectPhysicalHeightPixels =
            objectDigitalCorners_px[objectDigitalCorners_px.length - 1]
                - objectDigitalCorners_px[0];
        // Return estimated range to object in meters.
        return (cameraCalFactor
            * ((objectPhysicalHeightMeters * cameraFocalLengthPixels)
                / objectPhysicalHeightPixels));
    }

    /**
     * Estimates robot's range to a target using the target's known height. Algorithm similar to
     * {@link org.photonvision.PhotonUtils} but also allows for camera installation yaw (at the
     * expense of accuracy, particularly when operating around the angular limits of the camera's
     * FOV). This method can produce more stable results than SolvePNP when well tuned, if the full
     * 6d robot pose is not required. Note that this method requires the camera to have 0 roll (not
     * be skewed clockwise or CCW relative to the floor), and for there to exist a height
     * differential between goal and camera. The larger this differential, the more accurate the
     * distance estimate will be. For small differentials, use rangeToTarget_FocalLength.
     *
     * @param targetObservation A data type containing vision pipeline results for a single target.
     *        Used to determine the pitch & yaw of the target from the centerline of the camera's
     *        lens in degrees. Centerline assumed through geometric center of conical FOV. Target
     *        pitch is positive above centerline; target yaw is positive right of centerline.
     * @param cameraTransform Transform3d of the camera relative to the robot. Used to determine the
     *        camera's height off the ground, the range offset, installation pitch, and installation
     *        yaw.
     * @param targetHeightMeters The physical height of the target off the floor as measured by the
     *        location of the detection reticle in meters. For example, if your detection reticle is
     *        set to the center of the bounding box, this height should be the elevation off the
     *        ground to the center of the target.
     * @param cameraCalFactor An empirical calibration factor to account for real lens effects (e.g.
     *        blur, distortion, focus).
     * @param cameraOffset An empirical calibration factor to account for bias in the range estimate
     *        as a result of either camera hardware or installation.
     * @return The estimated robot range to the target in meters.
     */
    public double rangeToTarget_Pitch(TargetObservation targetObservation,
        Transform3d cameraTransform, double targetHeightMeters, double cameraCalFactor,
        double cameraOffset)
    {
        // Empirically-determined tolerance (m)
        // Below which, height differential is too small for algorithm to be reliable.
        double tolerance = 0.175;
        // Salient camera transform parameters
        // X offset of camera from robot center (apply as offset to range calculation).
        double cameraRangeDelta = cameraTransform.getX();
        // Z offset of camera from robot center (used within range calculation).
        double cameraHeightMeters = cameraTransform.getZ();
        // Camera installation pitch math uses positive up but .getY() is positive down.
        double cameraPitchRadians = -cameraTransform.getRotation().getY();
        // Camera installation yaw math uses positive left and .getZ() uses the same.
        double cameraYawRadians = cameraTransform.getRotation().getZ();
        // When camera yaw is applied, the pitch triangle becomes non-orthogonal, requiring a
        // correction.
        double yawCorrection;
        if (Math.abs(targetHeightMeters - cameraHeightMeters) > tolerance) {
            if (cameraYawRadians == 0) {
                // Mathematically verified for camera pitched up or down with target above or below
                // lens centerline. This is the most robust configuration.
                // Mathematical approach: object projection onto lens centerline plane.
                yawCorrection = 1;
            } else {
                // Empirical, algebraic correction; it's workable but accuracy is relatively limited
                // and further degrades at extreme angles (relative to the camera's FOV). Pose
                // estimation or vector transforms are more appropriate but not implemented.
                // Mathematical approach: lens centerline projection onto object/lens-center plane.
                yawCorrection =
                    Math.cos(Math.abs(cameraYawRadians - Math.toRadians(targetObservation.yaw())));
            }
            return (((targetHeightMeters - cameraHeightMeters)
                / Math.tan(cameraPitchRadians + Math.toRadians(targetObservation.pitch())))
                * yawCorrection * cameraCalFactor + cameraOffset + cameraRangeDelta);

        } else {
            // Use rangeToTarget_FocalLength.
            return -1.0d;
        }
    }

    /**
     * Estimates robot's heading to a target using the target's robot-relative calculated range.
     * Allows for camera installation yaw (at the expense of accuracy, particularly when operating
     * around the angular limits of the camera's FOV). Note that this method requires the camera to
     * have 0 roll (not be skewed clockwise or CCW relative to the floor), and for there to exist a
     * finite range between goal and camera. The larger this differential, the more accurate the
     * distance estimate will be.
     *
     * @param targetObservation A data type containing vision pipeline results for a single target.
     *        Used to determine the yaw of the target from the centerline of the camera's lens in
     *        degrees. Target pitch is positive above centerline; target yaw is positive right of
     *        centerline.
     * @param cameraTransform Transform3d of the camera relative to the robot. Used to determine the
     *        camera's heading offset and installation yaw.
     * @param targetRangeMeters Robot's range to the target in meters.
     * @param cameraCalFactor An empirical calibration factor to account for real lens effects (e.g.
     *        blur, distortion, focus).
     * @param cameraOffset An empirical calibration factor to account for bias in heading estimate
     *        as a result of either camera or installation.
     * @return The estimated robot heading to the target in meters.
     */
    public double headingToTarget_Yaw(TargetObservation targetObservation,
        Transform3d cameraTransform, double targetRangeMeters, double cameraCalFactor,
        double cameraOffset)
    {
        // Salient camera transform parameters
        // Camera's range to target (math utilizes camera's range, not robot's).
        double cameraRangeMeters = targetRangeMeters - cameraTransform.getX();
        // Y offset of camera from robot center (apply as offset to heading calculation).
        double cameraHeadingDelta = cameraTransform.getY();
        // Camera installation yaw math uses positive left and .getZ() uses the same.
        double cameraYawRadians = cameraTransform.getRotation().getZ();
        // Mathematically verified for target left or right of centerline & camera yawed left or
        // right; no sign correction required.
        // Positive heading = robot right of target; negative heading = robot left of target.
        return ((Math.tan(cameraYawRadians - Math.toRadians(targetObservation.yaw())))
            * cameraRangeMeters * cameraCalFactor + cameraOffset + cameraHeadingDelta);
    }

    /**
     * Estimates the target's 2d distance from the robot using target's robot-relative range &s
     * heading.
     *
     * @param targetRangeMeters Robot's range to the target in meters.
     * @param targetHeadingMeters Robot's heading to the target in meters.
     * @return The estimated 2d distance from the robot to the target in meters.
     */
    public double distanceToTarget2d(double targetRangeMeters, double targetHeadingMeters)
    {
        // Distance from robot to target
        return Math.sqrt((Math.pow(targetRangeMeters, 2) + Math.pow(targetHeadingMeters, 2)));
    }

    /**
     * Estimates a {@link Translation2d} that maps the detected target's position in field
     * coordinates. This function effectively transforms the robot's pose by its local dX (range) &
     * dY (heading) to the target; the robot local coodinate frame is the one attached to the robot
     * "center" (the point upon which the camera transform is applied to). Note that the gyro angle
     * provided *must* line up with the field coordinate system -- that is, it should read zero
     * degrees when pointed towards the opposing alliance station, and increase as the robot rotates
     * CCW.
     *
     * @param targetRangeMeters Robot's range to the target is meters (dX in robot local).
     * @param targetHeadingMeters Robot's heading to the target in meters (dY in robot local).
     * @param robotPose The 2D pose of the robot on the field.
     * @return A Translation2d of the detected object in field coordinates.
     */
    public Translation2d estimateTargetToField(double targetRangeMeters,
        double targetHeadingMeters, Pose2d robotPose)
    {
        Translation2d fieldToTargetTranslation = robotPose
            .transformBy(new Transform2d(targetRangeMeters, targetHeadingMeters, new Rotation2d()))
            .getTranslation();
        return fieldToTargetTranslation;
    }

    /**
     * Generates an N-element FIFO list of the last N objects detected by the robot. to-do: doc
     * string
     */
    public void getLastNDetections(int N,
        ArrayList<Translation2d> lastNDetections, double toleranceMeters,
        Translation2d targetTranslation)
    {
        Translation2d currentTranslation;
        boolean isNewDetection = true;
        for (int i = 0; i < lastNDetections.size(); i++) {
            currentTranslation = lastNDetections.get(i);
            if ((Math.abs(targetTranslation.getX() - currentTranslation.getX()) <= toleranceMeters)
                && (Math
                    .abs(
                        targetTranslation.getY() - currentTranslation.getY()) <= toleranceMeters)) {
                isNewDetection = false;
            }
        }
        if (isNewDetection && lastNDetections.size() >= N) {
            lastNDetections.remove(0);
            lastNDetections.add(N - 1, targetTranslation);
        } else if (isNewDetection) {
            lastNDetections.add(targetTranslation);
        }
    }

    // To-do: Add isConnected method. quickly re-verify equations. test results in sim. begin
    // writing simple unit tests.
}
