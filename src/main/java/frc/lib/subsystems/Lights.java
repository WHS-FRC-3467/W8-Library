// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;

import frc.lib.io.lights.LightsIO;

/**
 * Class for simplified Lights implementation
 */
public class Lights {
    private final LightsIO io;

    /**
     * Constructs Lights.
     *
     * @param io the IO to interact with.
     */
    public Lights(LightsIO io)
    {
        this.io = io;
    }

    /**
     * Passes ControlRequest to IO layer
     *
     * @param request {@link ControlRequest}
     */
    public void setAnimation(ControlRequest request)
    {
        io.setAnimation(request);
    }

    /**
     * Passes ControlRequests to IO layer
     *
     * @param request {@link ControlRequest}
     */
    public void setAnimations(ControlRequest[] requests)
    {
        for (ControlRequest request : requests) {
            io.setAnimation(request);
        }
    }
}
