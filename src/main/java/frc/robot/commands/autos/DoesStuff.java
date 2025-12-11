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

package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.FeetPerSecond;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.AutoCommand;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.linear.Linear;
import frc.robot.subsystems.linear.Linear.Setpoint;
import frc.robot.util.BallSimulator;

public class DoesStuff extends AutoCommand {
    PathPlannerPath start = null;
    PathPlannerPath mid = null;

    public DoesStuff(Drive drive, Linear linear, Flywheel flywheel)
    {
        try {
            start = PathPlannerPath.fromPathFile("start - mid");
            mid = PathPlannerPath.fromPathFile("mid - end");
            Mode real = Constants.currentMode.REAL;
            if (Robot.isSimulation() && !Logger.hasReplaySource()) {
                addCommands(AutoCommands.resetOdom(drive, start));
                addCommands(Commands.sequence(AutoBuilder.followPath(start),
                    linear.setGoal(Setpoint.RAISED),
                    Commands.waitSeconds(2),
                    AutoBuilder.followPath(mid),
                    flywheel.shoot(),
                    Commands.runOnce(() -> BallSimulator.launch(FeetPerSecond.of(15.0),
                        RobotState.getInstance()))


                ));
            }
        } catch (Exception e) {
            DriverStation.reportError(
                "Path Failed to Load in " + this.getName() + " " + e.getMessage(),
                e.getStackTrace());
            throw new RuntimeException("Path Failed to Load in " + this.getName());
        }
    }

    public List<Pose2d> getAllPathPoses()
    {
        return Stream.of(start.getPathPoses(), mid.getPathPoses())
            .flatMap(Collection::stream)
            .collect(Collectors.toList());
    }

    @Override
    public Pose2d getStartingPose()
    {
        return start.getStartingHolonomicPose().get();
    }
}
