// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.LightsIO;

public class LightsSubsystem<IO extends LightsIO> extends SubsystemBase {
    protected final IO io;
    protected final String name;

    public LightsSubsystem(String name, IO io) {
        super(name);
        this.io = io;
        this.name = name;
    }

    @Override
    public void periodic() {
    }

    public void setAnimation(ControlRequest request) {
        io.setAnimation(request);
    }

    public Command setAnimationCommand(ControlRequest request) {
        return run(() -> setAnimation(request));
    }

    public Command setAnimationCommand(ControlRequest[] requests) {
        return run(() -> {
            for (ControlRequest request : requests) {
                setAnimation(request);
            }

        });
    }
}
