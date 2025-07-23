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

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.drive.DriveConstants;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static final boolean tuningMode = true;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final class PathConstants {

        public static final double PATHGENERATION_DRIVE_TOLERANCE = 0.02; // 2 cm tolerance for pathfinding

        public static final Pose2d 
            PATHFIND_TO_POSE_1_TARGET = new Pose2d(3, 7, Rotation2d.kZero);
        public static final Pose2d 
            PATHFIND_TO_POSE_2_TARGET = new Pose2d(3, 3, Rotation2d.kZero);

        public static final Pose2d ON_THE_FLY_PATH_1_TARGET = new Pose2d(6, 6, Rotation2d.k180deg);
        public static final Pose2d ON_THE_FLY_PATH_2_TARGET = new Pose2d(6, 6, Rotation2d.k180deg);

        public static final ArrayList<Pose2d> ON_THE_FLY_PATH_2_WAYPOINTS = new ArrayList<>(Arrays.asList(
            new Pose2d(1, 1, Rotation2d.kCCW_90deg), 
            new Pose2d(2, 5, Rotation2d.kCW_90deg), 
            new Pose2d(5, 2, Rotation2d.k180deg)
        ));

        // Tune the maxAcceleration, maxAngularVelocityRadPerSec, and maxAngularAccelerationRacPerSecSq constraints for pathfinding
        public static final PathConstraints ON_THE_FLY_PATH_CONSTRAINTS = new PathConstraints(
            DriveConstants.kSpeedAt12Volts.magnitude(), 
            4.0, 
            Units.degreesToRadians(540), 
            Units.degreesToRadians(720));
    }
}
