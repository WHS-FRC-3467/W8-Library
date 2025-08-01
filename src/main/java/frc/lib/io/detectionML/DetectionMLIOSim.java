// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.io.detectionML;

/** Add your docs here. */
public class DetectionMLIOSim implements DetectionMLIO {
    protected final String cameraName;

    public DetectionMLIOSim(String cameraName)
    {
        this.cameraName = cameraName;

    }

    @Override
    public void updateInputs(DetectionMLIOInputs inputs)
    {}

    @Override
    public String getCamera()
    {
        return cameraName;
    }

}
