// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.devices;

import org.littletonrobotics.junction.Logger;
import frc.lib.io.detectionML.DetectionMLIO;
import frc.lib.io.detectionML.DetectionMLIOInputsAutoLogged;
import frc.lib.io.detectionML.DetectionMLIO.TargetObservation;

/** Add your docs here. */
public class DetectionML {
    private final DetectionMLIO io;
    private final DetectionMLIOInputsAutoLogged inputs = new DetectionMLIOInputsAutoLogged();

    public DetectionML(DetectionMLIO io)
    {
        this.io = io;
    }

    public void periodic()
    {
        io.updateInputs(inputs);
        Logger.processInputs(io.getCamera(), inputs);
    }

    public TargetObservation[] getTargetObservations()
    {
        return inputs.latestTargetObservations;
    }

    // TODO: Add isConnected method
    // Place to add device level methods

}
