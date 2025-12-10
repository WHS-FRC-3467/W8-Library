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

import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.RobotState;

public class ProjectileAnalyzer {
    
    // Constants
    private static final double LITTLE_G = 9.80665; // Magnitude of acceleration due to gravity (m/s^2)
    private static final double MIN_ANGLE = 20.0; // Minimum launch angle in degrees
    private static final double MAX_ANGLE = 70.0; // Maximum launch angle in degrees
    private static final double DT = 0.02; // Delta Time or Time step
    private static final double SHOT_TOLERANCE = 0.02; // Tolerance for hitting the target, in meters for each component
    private static final double AIR_DENSITY = 1.225; // kg/m^3 at sea level
    private static final double DRAG_COEFFICIENT = 0.45;
    private static final double CROSS_SECTIONAL_AREA = Math.PI * Math.pow(0.1, 2); // Example diameter of 0.2m
    private static final double PROJECTILE_MASS = 0.68; // Example mass in kg
    private static final double LIFT_COEFFICIENT = 0.2; // Example Magnus effect coefficient - need to experimentally determine
    private static final double SPIN_RATE = 1; // The how much spin is on the projectile, in revolutions per second
    private static final boolean SPIN = false; // Whether to include Magnus effect in simulation

    public record Shot(double angle, double magnitude) {}

    /**
     * Returns a list containing arm and flywheel targets based on physics simulation for a provided
     * distance and target height.
     *
     * @param distance Horizontal distance to the target in meters
     * @return ArrayList<Double> containing the optimal launch angle (degrees) and velocity (m/s)
     */
    public static Shot getPresetShot(double distance) {

        // Height difference between release point and target
        // z of 3D pose of target, in meters. 0.8 is a placeholder for the mechanism release height
        double deltaH = RobotState.getInstance().getHeightToTarget().in(Meters);

        // Iterate over possible launch angles to find the optimal solution
        double optimalAngle = 0.0;
        double optimalVelocity = 30; // Start with a very high velocity
        double minScore = Double.MAX_VALUE; // Lower score = better solution

        for (double angle = MIN_ANGLE; angle <= MAX_ANGLE; angle += 1.0) { // Test angles from 20° to 70°
            double radians = Math.toRadians(angle); // Convert angle to radians

            // Calculate the required initial velocity magnitude for this angle
            double speed = calculateLaunchSpeed(distance, deltaH, radians);

            if (speed > 0) { // Don't bother to consider angles that don't have a working magnitude
                // Score the solution based on velocity and angle
                double score = calculateScore(speed, angle);

                // Keep the solution with the lowest score
                if (score < minScore) {
                    minScore = score;
                    optimalAngle = angle;
                    optimalVelocity = speed;
                }
            }
        }

        return new Shot(optimalAngle, optimalVelocity);
    }

    /**
     * Simulates projectile motion with drag and Magnus effect to find the required launch speed for a given angle.
     *
     * @param distance Horizontal distance to the target in meters
     * @param height   Vertical height difference to the target in meters
     * @param angle    Launch angle in radians
     * @return Required launch speed in m/s, or -1 if no valid solution is found
     */
    private static double calculateLaunchSpeed(
            double distance, double height, double angle) {

        // Initial speed - try from 1 m/s to 18 m/s
        double velocity = 1.0;

        while (velocity <= 18.0) {
            // Decompose initial velocity vector
            // all horizontal x and y direction velocity will be x to simplify things
            // So essentially we are looking at the up component and the forward/towards target component
            double vx = velocity * Math.cos(angle);
            double vz = velocity * Math.sin(angle);

            // Let the point of release be the "zero" position
            // So target is at (distance, height), relative to the zero point
            double x = 0, z = 0;

            // Declare vectors for Magnus effect calculation
            Vector<N3> velocityVector;
            Vector<N3> angularVelocityVector;
            Vector<N3> magnusDirectionVector;
            Vector<N3> magnusForceVector;

            // Simulate until the projectile hits the ground or passes the target
            while (z >= 0 && x < distance + SHOT_TOLERANCE) { 
                // Calculate forces
                double speed = Math.sqrt(vx * vx + vz * vz);
                double dragForce = 0.5 * DRAG_COEFFICIENT * AIR_DENSITY * CROSS_SECTIONAL_AREA * speed * speed;
                // Decompose drag force into components
                double dragX = -dragForce * (vx / speed);
                double dragZ = -dragForce * (vz / speed);
                if (SPIN) {
                    /* Calculate the Magnus effect force
                     * Magnus Force = (1/2) * A * CL * ​(ω×v)
                     * A = Cross-sectional area
                     * CL = Lift coefficient (determined experimentally)
                     * ​ω×v = Cross product of angular velocity vector * linear velocity vector
                     */
                    velocityVector = new Vector<>(new Translation3d(vx, 0.0, vz).toVector());
                    angularVelocityVector = new Vector<>(new Translation3d(0.0, SPIN_RATE * 2 * Math.PI, 0.0).toVector()); // Convert rev/s to rad/s
                    magnusDirectionVector = Vector.cross(velocityVector, angularVelocityVector);
                    magnusForceVector = magnusDirectionVector.times(0.5 * CROSS_SECTIONAL_AREA * LIFT_COEFFICIENT);
                    // Break magnus force into x and z components. It is technically normal to the velocity vector
                    double magnusX = magnusForceVector.get(0); // Magnus force in x-direction (horizontal towards target)
                    double magnusZ = magnusForceVector.get(2); // Magnus force in z-direction (upward lift)

                    // Update velocity
                    vx += (dragX + magnusX) / PROJECTILE_MASS * DT; // delta v = F/m * delta t
                    vz += (dragZ + magnusZ - LITTLE_G) / PROJECTILE_MASS * DT;
                } else {
                    // Update velocity
                    vx += dragX / PROJECTILE_MASS * DT; // delta v = F/m * delta t
                    vz += (dragZ - LITTLE_G) / PROJECTILE_MASS * DT;
                }
                // Update position
                x += vx * DT;
                z += vz * DT;

                // Check if the projectile hits the target - or reaches a point close enough
                if (Math.abs(x - distance) < SHOT_TOLERANCE && Math.abs(z - height) < SHOT_TOLERANCE) {
                    return Math.sqrt(vx * vx + vz * vz);
                }
            }
            velocity += 0.1; // Increment velocity and try again
        }
        return -1; // No valid solution
    }
    
    /**
     * Scores a velocity vector based on magnitude and direction. Lower scores are better.
     *
     * @param magnitude Launch velocity in m/s
     * @param angle    Launch angle in degrees
     * @return A score representing the desirability of the velocity
     */
    private static double calculateScore(double magnitude, double angle) {
        // Prioritize lower velocity and lower angles
        double anglePenalty = Math.sin(Math.toRadians(angle)) * 5; // Penalize high angles
        return magnitude * anglePenalty;
    }
}
