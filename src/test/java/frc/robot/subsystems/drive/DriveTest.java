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

package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.Timer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.junit.Assert.fail;

public class DriveTest implements AutoCloseable {
    static final double DELTA = 1e-2; // acceptable deviation range
    Drive drive;
  
    @BeforeEach // this method will run before each test
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        drive = new Drive(new GyroIO() {},
            new ModuleIOSim(DriveConstants.FrontLeft),
            new ModuleIOSim(DriveConstants.FrontRight),
            new ModuleIOSim(DriveConstants.BackLeft),
            new ModuleIOSim(DriveConstants.BackRight));

        /* enable the robot */
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        /* delay ~100ms so the devices can start up and enable */
        Timer.delay(0.100);
    }
  
    @SuppressWarnings("PMD.SignatureDeclareThrowsException")
    @AfterEach // this method will run after each test
    void shutdown() throws Exception {
        close();
    }
  
    @Test
    public void robotIsEnabled() {
        /* verify that the robot is enabled */
        try {
        assertTrue(DriverStation.isEnabled());
        } catch (Exception e) {
            fail("Robot is not enabled: " + e.getMessage());
        }
    }

    // TODO: Decide whether to keep this test
    @Test // marks this method as a test
    void testStop() {
        /* set the voltage supplied by the battery */
        drive.setSupplyVoltage(RobotController.getBatteryVoltage());

        drive.stop();
        Timer.delay(1.0);
        try {
            assertEquals(0.0, drive.getFFCharacterizationVelocity(), DELTA); // make sure that the speed of the motor is 0
        } catch (Exception e) {
            fail("Failed to stop the drive: " + e.getMessage());
        }
    }
  
    @Test
    void testDriveVelocity() {
        /* set the voltage supplied by the battery */
        drive.setSupplyVoltage(RobotController.getBatteryVoltage());

        drive.runVelocity(new ChassisSpeeds(3.0, 3.0, 0.0));
        Timer.delay(2.0);
        try {
            assertEquals(new ChassisSpeeds(3.0, 3.0, 0.0), drive.getChassisSpeeds(), DELTA);
        } catch (Exception e) {
            fail("Failed to run drive linear velocity of 3 m/s in the x direction and 3 m/s in the y direction: " + e.getMessage());
        }
    }
  
    @Test
    void testSteerVelocity() {
        /* set the voltage supplied by the battery */
        drive.setSupplyVoltage(RobotController.getBatteryVoltage());

        drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 1.5));
        Timer.delay(1.0);
        try {
            assertEquals(new ChassisSpeeds(0.0, 0.0, 1.5), drive.getChassisSpeeds(), DELTA);
        } catch (Exception e) {
            fail("Failed to run drive rotational velocity of 1.5 rad/s: " + e.getMessage());
        }
    }
  
    // TODO: Decide whether to keep this test
    @Test
    void testX() {
        /* set the voltage supplied by the battery */
        drive.setSupplyVoltage(RobotController.getBatteryVoltage());

        drive.stopWithX();
        Timer.delay(1.0);
        try {
            SwerveModulePosition[] swerveModulePositions = drive.getModulePositions();
            Rotation2d[] targetAngles = {
                new Rotation2d(-Math.atan(DriveConstants.FrontLeft.LocationY/DriveConstants.FrontLeft.LocationX)), // Front Left
                new Rotation2d(Math.atan(DriveConstants.FrontRight.LocationY/DriveConstants.FrontRight.LocationX)), // Front Right
                new Rotation2d(-Math.atan(DriveConstants.BackLeft.LocationY/DriveConstants.BackLeft.LocationX)), // Back Left
                new Rotation2d(Math.atan(DriveConstants.BackRight.LocationY/DriveConstants.BackRight.LocationX)) // Back Right
            };
            // Test position of modules
            for (int i = 0; i < swerveModulePositions.length; i++) {
                assertEquals(targetAngles[i].getRadians(), swerveModulePositions[i].angle.getRadians(), DELTA);
            }
            assertEquals(0.0, drive.getFFCharacterizationVelocity(), DELTA); // make sure that the drivetrain reaches a velocity of zero

        } catch (Exception e) {
            fail("Failed to turn the Drive modules into an X arrangement to resist movement: " + e.getMessage());
        }
    }

    @Override
    public void close() {
       // do nothing for now
       // Goal is to close close motor controller resources
    }
  }