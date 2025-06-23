// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lasercan1;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import frc.lib.io.distancesensor.DistanceSensor;
import frc.lib.io.distancesensor.DistanceSensorLaserCAN;
import frc.lib.io.distancesensor.DistanceSensorSim;
import frc.robot.Ports;
import frc.robot.Robot;

/** Add your docs here. */
public class LaserCAN1Constants {

    public final static String name = "LaserCAN #1";
    private final static RangingMode rangingMode = RangingMode.SHORT;
    private final static RegionOfInterest roi = new RegionOfInterest(8, 8, 4, 4);
    private final static TimingBudget timingBudget = TimingBudget.TIMING_BUDGET_20MS;

    public static DistanceSensor getDistanceSensorIO() {
        if (Robot.isReal()) {
            return new DistanceSensorLaserCAN(Ports.laserCAN1, name, rangingMode, roi,
                    timingBudget);
        } else {
            return new DistanceSensorSim(name);
        }
    }
}
