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
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.rotary.Rotary;
import frc.robot.util.LookUpTable;
import frc.robot.util.RobotToTargetUtil;
import frc.robot.util.ShotSetpoint;

public class SmartAimAtTarget extends Command {
    
    private final Drive drive;
    private final Rotary rotary;
    private final Flywheel flywheel;
    private LookUpTable lookup = LookUpTable.getInstance();

     /**
     * Creates a new SmartAimAtTarget command, part of the ShootOnTheMove implementation.
     *
     * @param drive The drive subsystem used by this command.
     * @param rotary The rotary subsystem used by this command.
     * @param flywheel The flywheel subsystem used by this command.
     */
    public SmartAimAtTarget(Drive drive, Rotary rotary, Flywheel flywheel) {
        this.drive = drive;
        this.rotary = rotary;
        this.flywheel = flywheel;
        addRequirements(rotary, flywheel);
    }

    /** The initial subroutine of a command. Called once when the command is initially scheduled. */
    @Override
    public void initialize() {

    }

    /** The main body of a command. Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {
        Pose2d futurePose = getFuturePose(() -> RobotState.getInstance().getTimeToBeReady().getAsDouble()); // Look ahead 0.2 seconds
        // Determine where to aim based on predicted future pose
        ShotSetpoint setpoints = lookup.getShotData(RobotToTargetUtil.getDistanceToTarget(futurePose));
        // Set rotary angle & flywheel setpoint based on predicted future pose.
        // Hopefully the subsystems will "catch up" along the way.
        rotary.setSetpoint(Degrees.of(setpoints.getArmOutput()));
        flywheel.shoot(RotationsPerSecond.of(setpoints.getFlywheelOutput()));
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
     * POSTCONDITION: Command ends when both rotary and flywheel are at their setpoints. It is time to shoot!
     * @return whether the command has finished.
     */
    @Override
    public boolean isFinished() {
        ShotSetpoint setpoints = lookup.getShotData(RobotToTargetUtil.getDistanceToTarget(drive.getPose()));
        return rotary.nearGoal(Degrees.of(setpoints.getArmOutput())) 
            && flywheel.nearGoal(setpoints.getFlywheelOutput());
    }

    /**
     * Get the predicted future pose of the robot based on current velocity and acceleration.
     *
     * @param timeSecondsAhead Supplier that provides the time in seconds to look ahead.
     * @return Predicted future Pose3d of the robot.
     */
    public Pose2d getFuturePose(Supplier<Double> timeSecondsAhead) {
        double deltaT = timeSecondsAhead.get();
        Pose2d currentPose = RobotState.getInstance().getPose();
        double currentHeadingRad = currentPose.getRotation().getRadians();
        double vx = RobotState.getInstance().getFieldRelativeVelocity().vxMetersPerSecond;
        double vy = RobotState.getInstance().getFieldRelativeVelocity().vyMetersPerSecond;
        // The kinematics equations to calculate future position with constant acceleration
        double futureX = currentPose.getX() + vx * deltaT + 0.5 * drive.getAccelerationX() * deltaT * deltaT;
        double futureY = currentPose.getY() + vy * deltaT + 0.5 * drive.getAccelerationY() * deltaT * deltaT;
        return new Pose2d(futureX, futureY, new Rotation2d(currentHeadingRad));
    }
}