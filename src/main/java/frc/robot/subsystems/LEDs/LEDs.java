// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.io.LightsIO;
import frc.lib.subsystems.LightsSubsystem;

public class LEDs extends LightsSubsystem<LightsIO> {

    public LEDs() {
        super("LEDs", LEDsConstants.getLightsIO());
        SmartDashboard.putData("Auto LED Command", autoLED());
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public Command autoLED() {
        return setAnimationCommand(LEDsConstants.autoAnimation);
    }

    public Command setOff() {
        return setAnimationCommand(LEDsConstants.offAnimation);
    };

    public Command flashCANdle() {
        return setAnimationCommand(LEDsConstants.candleFlash);
    }

}
