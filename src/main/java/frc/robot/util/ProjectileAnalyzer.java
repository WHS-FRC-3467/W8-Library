/* Copyright (C) 2025 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

package frc.robot.util;

import java.util.ArrayList;

public class ProjectileAnalyzer {
    
    private double distance;
    private double releaseHeight;
    private double targetHeight;

    /**
     * 
     * @param distance horizontal distance, in meters from release point to target
     */
    public ProjectileAnalyzer(double distance) {
        this.distance = distance;
        this.releaseHeight = 0.0; // z of 3D pose of robot, in meters
        this.targetHeight = 2.0; // z of 3D pose of target, in meters
    }

    /**
     * Returns a list containing arm and flywheel targets based on physics simulation for a provided
     * @param distance
     * 
     * @return ArrayList<Double> containing arm and flywheel setpoints
     */
    public ArrayList<Double> getPresetShot() {

        // TODO: Implement physics simulation
        return new ArrayList<Double>();
    }
}
