// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lasercan1;

import static edu.wpi.first.units.Units.Millimeters;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.io.distancesensor.DistanceSensorIO;
import frc.lib.subsystems.DistanceSensor;
import lombok.NonNull;

/** Add your docs here. */
public class LaserCAN1 extends SubsystemBase { // Don't extend if contained in superstructure
    private final DistanceSensor distanceSensor;

    public final Trigger inside =
        new Trigger(() -> betweenDistance(Millimeters.of(5), Millimeters.of(10)));

    public LaserCAN1(DistanceSensorIO io)
    {
        distanceSensor = new DistanceSensor(io);
    }

    @Override
    public void periodic()
    {
        distanceSensor.periodic();
    }

    private boolean betweenDistance(Distance min, Distance max)
    {
        if (distanceSensor.getDistance().isEmpty()) {
            return false;
        }

        Distance distance = distanceSensor.getDistance().get();
        return distance.gt(min) && distance.lt(max);
    }
}
