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

import java.util.List;
import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

public class OnTheFlyPathCommand extends Command {

    PathPlannerPath path;
    Supplier<Pose2d> currentPose;
    List<Pose2d> waypointPoses;
    Pose2d targetPose;
    PathConstraints constraints;
    double goalEndVelocity;
    boolean shouldFlipPath;
    boolean shouldMirrorPath;
    double tolerance;
    Command command;

    /**
     * Automatically generates a PathPlanner path on-the-fly based on dynamic inputs.
     * @param drive The Drive subsystem to get the current pose from
     * @param waypointPoses A list of Pose2d waypoints to include in the path.
     * @param targetPose The final target pose for the path.
     * @param constraints The PathConstraints to apply to the path.
     * @param goalEndVelocity The final velocity in meters/sec.
     * @param shouldFlipPath Whether to flip the path from the blue/red alliance side of the field
     * @param shouldMirrorPath Whether to mirror the path from the left/right side of the field
     * @param tolerance The allowed tolerance, in meters, of the robot's position from the target pose.
     * Saves the path and command for retrieval to be executed by the CommandScheduler in Robot.java
     */
    public OnTheFlyPathCommand(Drive drive, Supplier<Pose2d> currentPose, List<Pose2d> waypointPoses,
    Pose2d targetPose, PathConstraints constraints, double goalEndVelocity, boolean shouldMirrorPath, double tolerance) {
        addRequirements(drive);
        this.currentPose = currentPose;
        this.waypointPoses = waypointPoses;
        this.targetPose = targetPose;
        this.constraints = constraints;
        this.goalEndVelocity = goalEndVelocity;
        this.shouldMirrorPath = shouldMirrorPath;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize() {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            currentPose.get(), targetPose); // Start Pose and End pose (need to include a second pose or else the code will crash)
        if (waypointPoses != null) {
            waypoints.remove(waypoints.size()-1); // If additional waypoints are provided remove the target pose before adding it back in
            waypoints.addAll(PathPlannerPath.waypointsFromPoses(waypointPoses));
            waypointPoses.add(targetPose); // Add the target/last pose to the end of the waypoint pose list

        }

        PathConstraints pathConstraints = null;
        if (constraints == null) {
            pathConstraints = PathConstraints.unlimitedConstraints(12.0); // Unlimited constraints, only limited by motor torque and nominal battery voltage
        } else {
            pathConstraints = constraints;
        }
                
        // Create the path using the waypoints created above
        path = new PathPlannerPath(
            waypoints,
            pathConstraints,
            new IdealStartingState(0, currentPose.get().getRotation()), // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(goalEndVelocity, targetPose.getRotation()) // Goal end state. You can set a holonomic rotation here.
        );
        
        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = DriverStation.getAlliance().get() == Alliance.Blue;
        path = shouldMirrorPath ? path.mirrorPath() : path;
        // Start the pathfinding path tracker
        DriveCommands.setOnTheFlyPath((DriverStation.getAlliance().get() == Alliance.Blue) ? path : path.flipPath());   
        command = AutoBuilder.followPath(path);
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public boolean isFinished() {
        // Is the magnitude of the difference between the current pose and the target pose (last pose of the path) less than the tolerance?
        return currentPose.get().minus(path.getPathPoses().get(path.getPathPoses().size()-1)).getTranslation().getNorm() < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        DriveCommands.setOnTheFlyPath(null);
        command.end(interrupted);
    }
    
}
