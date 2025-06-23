// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.distancesensor;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.io.distancesensor.DistanceSensor;
import frc.lib.io.distancesensor.DistanceSensorLaserCAN;
import frc.lib.io.distancesensor.DistanceSensorSim;
import frc.robot.Ports;
import frc.robot.Robot;

/** Add your docs here. */
public class DistanceSensorConstants {

    private final static RangingMode rangingMode = RangingMode.SHORT;
    private final static RegionOfInterest roi = new RegionOfInterest(8, 8, 4, 4);
    private final static TimingBudget timingBudget = TimingBudget.TIMING_BUDGET_20MS;

    public static DistanceSensor getDistanceSensor() {
        if (Robot.isReal()) {
            try {
                return new DistanceSensorLaserCAN(Ports.laserCAN1, "Distance Sensor #1", rangingMode, roi,
                        timingBudget);
            } catch (Exception e) {
                SmartDashboard.putString("Distance Sensor #1", "Failed");
                return new DistanceSensorSim("Distance Sensor #1");
            }
        } else {
            return new DistanceSensorSim("Distance Sensor #1");
        }
    }
}
