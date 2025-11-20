/*
 * Copyright (C) 2025 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */

package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import edu.wpi.first.units.measure.Distance;

public class LookUpTable {
    ShotSetpointList knownShots;

    private static LookUpTable instance = new LookUpTable();

    public static LookUpTable getInstance()
    {
        return instance;
    }

    public LookUpTable()
    {
        // Physics simulate!
        

        // Create the list of shot setpoints
        knownShots = new ShotSetpointList();
        knownShots.getShots().add(new ShotSetpoint(1.5, 12.71, 70));
        knownShots.getShots().add(new ShotSetpoint(2.0, 21.00, 70));
        knownShots.getShots().add(new ShotSetpoint(2.5, 24.89, 70));
        knownShots.getShots().add(new ShotSetpoint(3.0, 29.00, 70));
        knownShots.getShots().add(new ShotSetpoint(3.5, 31.20, 70)); // Ended Here
        knownShots.getShots().add(new ShotSetpoint(4.0, 32.50, 70)); // 32.10
        knownShots.getShots().add(new ShotSetpoint(4.5, 34.00, 75)); // 33.80
        knownShots.getShots().add(new ShotSetpoint(5.0, 35.00, 75)); // 32.50
        Collections.sort(knownShots.getShots());
    }

    /**
     * Obtains a shooter preset from a given target distance
     * 
     * @param distance measured distance to the shooting target
     * 
     * @return new shooter preset for given distance
     */
    public ShotSetpoint getShotData(Distance distance)
    {
        int endIndex = knownShots.getShots().size() - 1;
        double distanceFromTarget = distance.in(Meters);
        /*
         * Check if distance falls below the shortest distance in the lookup table. If the measured
         * distance is shorter select the lookup table entry with the shortest distance
         */
        if (distanceFromTarget <= knownShots.getShots().get(0).getDistance()) {
            return knownShots.getShots().get(0);
        }

        /*
         * Check if distance falls above the largest distance in the lookup table. If the measured
         * distance is larger select the lookup table entry with the largest distance
         */
        if (distanceFromTarget >= knownShots.getShots().get(endIndex).getDistance()) {
            return knownShots.getShots().get(endIndex);
        }
        /*
         * If the measured distance falls somewhere within the lookup table perform a binary seqarch
         * within the lookup table
         */
        return binarySearchDistance(knownShots.getShots(), 0, endIndex,
            distanceFromTarget);
    }

    /**
     * Perform fast binary search to find a matching shooter preset. if no matching preset is found
     * it interpolates a new shooter preset based on the two surrounding table entries.
     * 
     * @param shots: the table containing the shooter presets
     * 
     * @param startIndex: Starting point to search
     * 
     * @param endIndex: Ending point to search
     * 
     * @param Distance: Distance for which we need to find a preset
     * 
     * @return (Interpolated) shooting preset
     */
    private ShotSetpoint binarySearchDistance(List<ShotSetpoint> shots, int startIndex,
        int endIndex, double distance)
    {
        int mid = startIndex + (endIndex - startIndex) / 2;
        double midIndexDistance = shots.get(mid).getDistance();

        // If the element is present at the middle
        // return itself
        if (distance == midIndexDistance) {
            return shots.get(mid);
        }
        // If only two elements are left
        // return the interpolated config
        if (endIndex - startIndex == 1) {
            double percentIn =
                (distance - knownShots.getShots().get(startIndex).getDistance()) /
                    (knownShots.getShots().get(endIndex).getDistance() -
                        knownShots.getShots().get(startIndex).getDistance());
            return interpolateShotData(knownShots.getShots().get(startIndex),
                knownShots.getShots().get(endIndex), percentIn);
        }
        // If element is smaller than mid, then
        // it can only be present in left subarray
        if (distance < midIndexDistance) {
            return binarySearchDistance(shots, startIndex, mid, distance);
        }
        // Else the element can only be present in right subarray
        return binarySearchDistance(shots, mid, endIndex, distance);
    }

    /**
     * Obtain a new shooter preset by interpolating between two existing shot presets
     * 
     * Right now, the only supported interpolation is linear.
     * 
     * @param startPreset: Starting preset for interpolation
     * 
     * @param endPreset: Ending preset for interpolation
     * 
     * @param percentIn: Amount of percentage between the two values the new preset needs to be
     * 
     * @return new interpolated shooter preset
     */
    private ShotSetpoint interpolateShotData(ShotSetpoint startPreset,
        ShotSetpoint endPreset, double percentIn)
    {
        double distance = startPreset.getDistance()
            + (endPreset.getDistance() - startPreset.getDistance()) * percentIn;

        double armAngle = startPreset.getArmOutput()
            + (endPreset.getArmOutput() - startPreset.getArmOutput()) * percentIn;

        double flywheelOutput = startPreset.getFlywheelOutput()
            + (endPreset.getFlywheelOutput() - startPreset.getFlywheelOutput()) * percentIn;

        return new ShotSetpoint(distance, armAngle, flywheelOutput);
    }

    /**
     * MAKE SURE YOU SORT THE LIST BEFORE CALLING THIS FUNCTION
     * 
     * @param knownShots a sorted shooter config
     */
    public void setShotList(ShotSetpointList knownShots)
    {
        this.knownShots = knownShots;
    }

    /**
     * Returns a list of all known shot presets based on physics simulation
     * 
     * @return ArrayList of all known ShotSetpoints
     */
    public ArrayList<ShotSetpoint> getShotPresets() {


        return new ArrayList<ShotSetpoint>(knownShots.getShots());
    }
}
