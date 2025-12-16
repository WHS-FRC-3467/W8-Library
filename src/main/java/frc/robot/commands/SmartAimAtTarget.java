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

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.ProjectileAnalyzer;

public class SmartAimAtTarget extends Command {
    
    private static final double ROTATION_TOLERANCE = 5.0; // degrees
    private static final double MAX_ANGULAR_ROBOT_VELOCITY = 0.7; // radians per second
    private static final boolean PHYSICS_BASED_LOOKUP = true; // Whether to use physics-based lookup tables or pre-defined ones

    // Lookup table for angle (degrees) and speed (angular velocity of flywheel) based on distance to target
    private static final InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap magnitudeMap = new InterpolatingDoubleTreeMap();
    static{
        // Create the list of shot setpoints
        // For every 3 ish inches (0.05 meters) from target create a new preset:
        if (PHYSICS_BASED_LOOKUP) {
            for (double distance = 1.5; distance < 5; distance+= 0.05) {
                // Use Projectile Analyzer to get the vector
                // Angle in degrees, Speed in m/s
                ProjectileAnalyzer.Shot shot = ProjectileAnalyzer.getPresetShot(distance);
                // Magnitude Map - convert linear speed to angular speed of flywheel (rad/s) by dividing by flywheel radius
                magnitudeMap.put(distance, shot.magnitude()/FlywheelConstants.FLYWHEEL_RADIUS.in(Meters));
                // Assume arm angle is the same as the shot angle for now
                angleMap.put(distance, shot.angle());
            }
        } else {
            // Pre-defined lookup table values
            magnitudeMap.put(1.0, 325.0);   // Use rad/s for continuity with MotorIO logging
            angleMap.put(1.0, 55.0);        // degrees
            magnitudeMap.put(1.5, 325.0);
            angleMap.put(1.5, 50.0);
            magnitudeMap.put(2.0, 325.0);
            angleMap.put(2.0, 45.0);
            magnitudeMap.put(2.5, 325.0);
            angleMap.put(2.5, 41.0);
            magnitudeMap.put(3.0, 325.0);
            angleMap.put(3.0, 37.0);
            magnitudeMap.put(3.5, 400.0);
            angleMap.put(3.5, 33.0);
            magnitudeMap.put(4.0, 400.0);
            angleMap.put(4.0, 30.0);
            magnitudeMap.put(4.5, 400.0);
            angleMap.put(4.5, 27.0);
            magnitudeMap.put(5.0, 400.0);
            angleMap.put(5.0, 24.0);
        }
    }

    private final Drive drive;
    private final Superstructure superstructure;
    private final Flywheel flywheel;

     /**
     * Creates a new SmartAimAtTarget command, part of the ShootOnTheMove implementation.
     *
     * @param drive The drive subsystem used by this command.
     * @param rotary The rotary subsystem used by this command.
     * @param flywheel The flywheel subsystem used by this command.
     */
    public SmartAimAtTarget(Drive drive, Superstructure superstructure, Flywheel flywheel) {
        this.drive = drive;
        this.flywheel = flywheel;
        this.superstructure = superstructure;
        addRequirements(superstructure, flywheel);
    }

    /** The initial subroutine of a command. Called once when the command is initially scheduled. */
    @Override
    public void initialize() {}

    /** The main body of a command. Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {
        Pose2d futurePose = getFuturePose(() -> RobotState.getInstance().getTimeToBeReady().getAsDouble()); // Look ahead 0.2 seconds
        double distanceToTarget = RobotState.getInstance().getDistanceToTarget(futurePose).in(Meters);
    
        // Determine the desired angle and magnitude based on the distance to the target
        double angle = angleMap.get(distanceToTarget);
        double magnitude = magnitudeMap.get(distanceToTarget);
    
        // Use the superstructure to set the desired state
        superstructure.setGoal(Degrees.of(angle), Inches.of(0));
        flywheel.shoot(RotationsPerSecond.of(magnitude));
    }

    /**
     * The action to take when the command ends. Called when either the command finishes normally, or
     * when it interrupted/canceled.
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {}

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end()
     * method and un-schedule it.
     * POSTCONDITION: Command ends when both rotary and flywheel are at their setpoints, robot is facing the target, and drivetrain is not rotating too quickly. It is time to SCORE!
     * @return whether the command has finished.
     */
    @Override
    public boolean isFinished() {
        Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();
        double distanceToTarget = RobotState.getInstance().getDistanceToTarget(estimatedPose).in(Meters);
    
        // Determine the desired angle and magnitude based on the distance to the target
        double angle = angleMap.get(distanceToTarget);
        double magnitude = magnitudeMap.get(distanceToTarget);
    
        // Check if the superstructure is near the goal and the drivetrain is stable
        return superstructure.nearSetpoint(Degrees.of(angle), Inches.of(0)) && flywheel.nearGoal(magnitude)
            && MathUtil.isNear(RobotState.getInstance().getVelocity().omegaRadiansPerSecond, 0.0, MAX_ANGULAR_ROBOT_VELOCITY);
    }

    /**
     * Get the predicted future pose of the robot based on current velocity and acceleration.
     *
     * @param timeSecondsAhead Supplier that provides the time in seconds to look ahead.
     * @return Predicted future Pose3d of the robot.
     */
    public Pose2d getFuturePose(Supplier<Double> timeSecondsAhead) {
        double deltaT = timeSecondsAhead.get();
        Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
        double currentHeadingRad = currentPose.getRotation().getRadians();
        double vx = RobotState.getInstance().getFieldRelativeVelocity().vxMetersPerSecond;
        double vy = RobotState.getInstance().getFieldRelativeVelocity().vyMetersPerSecond;
        // The kinematics equations to calculate future position with constant acceleration
        double futureX = currentPose.getX() + vx * deltaT + 0.5 * drive.getAccelerationX() * deltaT * deltaT;
        double futureY = currentPose.getY() + vy * deltaT + 0.5 * drive.getAccelerationY() * deltaT * deltaT;
        return new Pose2d(futureX, futureY, new Rotation2d(currentHeadingRad));
    }
}