// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.subsystems;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.distancesensor.*;

public class DistanceSensorSubsystem extends SubsystemBase {
    protected final DistanceSensor sensor;
    protected final DistanceSensorInputsAutoLogged inputs = new DistanceSensorInputsAutoLogged();
    protected final String name;

    public DistanceSensorSubsystem(String name, DistanceSensor sensor)
    {
        super(name);
        this.sensor = sensor;
        this.name = name;
    }

    @Override
    public void periodic()
    {
        sensor.updateInputs(inputs);
        Logger.processInputs(name, inputs);
        SmartDashboard.putBoolean(name + " Connected?", inputs.connected);
        SmartDashboard.putNumber(name + " Distance (m): ", inputs.distance.in(Meters));
    }

    public Distance getDistance()
    {
        return inputs.distance;
    }

}
