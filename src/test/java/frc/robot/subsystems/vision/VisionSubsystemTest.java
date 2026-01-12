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

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.GeomUtil;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.TestUtil;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

class VisionSubsystemTest {

    private Drive drive;
    private VisionSubsystem visionSubsystem;
    private RobotState robotState;

    private int visionPoseCount = 0;

    @BeforeEach
    void setup()
    {
        assertTrue(HAL.initialize(500, 0));

        robotState = RobotState.getInstance();

        var tags = FieldConstants.APRILTAG_LAYOUT.getTags();
        assertTrue(!tags.isEmpty());

        var robotLocation =
            tags.get(0).pose
                .toPose2d()
                .plus(new Transform2d(2, 0, Rotation2d.k180deg));

        robotState.resetPose(robotLocation);

        visionSubsystem = VisionConstants.get();
        drive = DriveConstants.get();

        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        Timer.delay(0.100);
    }

    @Test
    void visionProcessingAddsVisionObservations()
    {
        visionPoseCount = 0;

        Transform3d tagToRobot =
            new Transform3d(1.0, 0.0, 0.0, new Rotation3d(Rotation2d.k180deg))
                // Move robot such that camera is perfectly facing the tag
                .plus(VisionConstants.FRONT_RIGHT_TRANSFORM.inverse());

        // Ensure the field has at least one tag
        var tags = FieldConstants.APRILTAG_LAYOUT.getTags();
        assertTrue(!tags.isEmpty(), "Expected at least one AprilTag in the field layout");

        robotState.resetPose(tags.get(0).pose.plus(tagToRobot).toPose2d());

        try {
            robotState.setVisionObservationConsumer(v -> {
                visionPoseCount++;
            });

            TestUtil.runTest(
                Commands.parallel(
                    Commands.idle(drive),
                    visionSubsystem.visionProcessingCommand()),
                2.0);

            assertTrue(
                visionPoseCount > 0,
                "Expected at least one vision pose observation");
        } catch (Exception e) {
            fail("Vision processing command failed: " + e.getMessage());
        }
    }

    @Test
    void cameraCalibrationCommandProducesCalibrationData()
    {
        int primaryIndex = 0;

        // Known transform from calibration target (tag) to robot center
        Transform3d knownRobotCenterTransform = new Transform3d(1.0, 0.0, 0.0, new Rotation3d());

        // Ensure the field has at least one tag
        var tags = FieldConstants.APRILTAG_LAYOUT.getTags();
        assertTrue(!tags.isEmpty(), "Expected at least one AprilTag in the field layout");

        robotState.resetPose(tags.get(0).pose.plus(knownRobotCenterTransform).toPose2d());
        TestUtil.runTest(Commands.run(() -> drive.runVelocity(new ChassisSpeeds(1.5, 1.5, 0.0))),
            1);

        // Run the calibration command for a short, deterministic number of loops
        // The calibration command uses SAMPLE_COUNT internally; in sim, ensure at least 1 sample
        // is collected
        TestUtil.runTest(
            visionSubsystem.cameraCalibrationCommand(primaryIndex, knownRobotCenterTransform),
            1.0); // 1 second is sufficient for sim

        // Grab the calibration result
        Transform3d measuredCamToRobot = VisionSubsystem.test;

        // Compute expected camera → robot transform
        // camera → robot = camera → target + target → robot
        // In simulation, assume camera → target is identity (or matches what the calibration
        // command will sample)
        Transform3d expectedCamToRobot = knownRobotCenterTransform;

        // Compare the transforms with tight tolerances
        boolean isClose = GeomUtil.isNear(
            measuredCamToRobot,
            expectedCamToRobot,
            Centimeter.one(),
            new Rotation3d(Degrees.one(), Degrees.one(), Degrees.one()));

        System.out.println("Actual camera → robot: " + measuredCamToRobot);
        System.out.println("Expected camera → robot: " + expectedCamToRobot);

        assertTrue(isClose, "Recorded camera-to-robot transform does not match expected value");
    }
}
