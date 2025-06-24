// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.lights.LightsIO;
import frc.lib.subsystems.Lights;

public class LEDs extends SubsystemBase {
    private final Lights lights;

    public LEDs(LightsIO io)
    {
        lights = new Lights(io);
    }

    @Override
    public void periodic()
    {
        lights.periodic();
    }

    public Command runDisabledAnimation()
    {
        return this.startEnd(
            () -> lights.setAnimations(LEDsConstants.disabledAnimation),
            () -> lights.setAnimations(LEDsConstants.offAnimation));
    }

    public Command runAutoAnimation()
    {
        return this.startEnd(
            () -> lights.setAnimations(LEDsConstants.autoAnimation),
            () -> lights.setAnimations(LEDsConstants.offAnimation));
    }
}
