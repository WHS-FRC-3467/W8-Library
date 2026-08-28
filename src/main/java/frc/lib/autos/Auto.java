// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;

import lombok.Getter;

import org.littletonrobotics.junction.Logger;

import java.util.List;

public class Auto {

    // Name of the auto
    @Getter private final String name;

    // Commands to be run
    @Getter private final Command command;

    // List of points for visual display on the dashboard
    @Getter private final List<Pose2d> points = List.of();

    public Auto(String name, Command command, List<Pose2d> points) {
        this.name = name;
        this.command = command;

        Logger.recordOutput("Autos/" + name, points.toArray(new Pose2d[0]));
    }

    // Returns starting pose of the first Pose2d
    public Pose2d getStartingPose() {
        if (points.isEmpty()) {
            return new Pose2d();
        }
        return points.get(0);
    }

    public Trajectory getTrajectory() {
        if (points.size() < 2) {
            return null;
        }
        // Create a trajectory from the points
        return TrajectoryGenerator.generateTrajectory(points, new TrajectoryConfig(4.0, 2.0));
    }
}
