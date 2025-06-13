// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.io;

import static edu.wpi.first.units.Units.Millimeters;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.CanDeviceId;

/** Add your docs here. */
public class DistanceIOLaserCAN extends DistanceIO {
    private final LaserCan laserCan;



    public DistanceIOLaserCAN(CanDeviceId id, String name, RangingMode rangingMode, RegionOfInterest regionOfInterest,TimingBudget timingBudget) {
        super(name);
        this.laserCan = new LaserCan(id.getDeviceNumber());
        //         while (!hasConfiged && tries < 5) {
        //     try {
        //         laserCan.setRangingMode(rangingMode);
        //         laserCan.setRegionOfInterest(regionOfInterest);
        //         laserCan.setTimingBudget(timingBudget);
        //         failedConfig.set(false);
        //         System.out.println("Succesfully configured " + name);
        //         hasConfiged = true;
        //     } catch (ConfigurationFailedException e) {
        //         System.out.println("Configuration failed for " + name + "! " + e);
        //         failedConfig.setText("Failed to configure " + name + "!");
        //         failedConfig.set(true);
        //         tries++;
        //     }
        // }

    }

    @Override
    public Distance getDistance() {
        // return Millimeters.of();
    }
}
