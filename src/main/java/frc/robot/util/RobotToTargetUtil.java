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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import lombok.Getter;
import lombok.Setter;

public class RobotToTargetUtil {
    
    public enum Target {
        // NAME(BLUE, RED)
        ONE(new Pose2d(Meters.of(0), Meters.of(0), Rotation2d.kZero), new Pose2d(FieldConstants.FIELDLENGTH, FieldConstants.FIELDLENGTH, Rotation2d.k180deg)),
        TWO(new Pose2d(), new Pose2d());

        @Getter
        private final Pose2d bluePose;

        @Getter
        private final Pose2d redPose;

        private Target(Pose2d blue, Pose2d red) {
            this.bluePose = blue;
            this.redPose = red;
        }

    }
   
    @Setter
    static Target target = Target.ONE;

    public static Distance getDistanceToTarget(Pose2d robotPose) {
        Translation2d robotTranslation = robotPose.getTranslation();
        Translation2d targetTranslation = (DriverStation.getAlliance().get() == Alliance.Blue ? target.bluePose.getTranslation() : target.redPose.getTranslation());
        return Meters.of(robotTranslation.getDistance(targetTranslation));
    }

    public static Distance getDistanceToTarget(Translation2d robotPose) {
        Translation2d targetTranslation = (DriverStation.getAlliance().get() == Alliance.Blue ? target.bluePose.getTranslation() : target.redPose.getTranslation());
        return Meters.of(robotPose.getDistance(targetTranslation));
    }

    public static Rotation2d getAngleToTarget(Translation2d robotPose) {
        return (DriverStation.getAlliance().get() == Alliance.Blue ? target.bluePose.getTranslation() : target.redPose.getTranslation()).minus(robotPose).getAngle();
    }

    public static Pose2d getTargetPose(Target target) {
        return (DriverStation.getAlliance().get() == Alliance.Blue ? target.getBluePose() : target.getRedPose());
    }
}
