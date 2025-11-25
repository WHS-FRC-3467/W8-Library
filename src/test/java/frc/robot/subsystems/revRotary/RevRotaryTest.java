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

package frc.robot.subsystems.revRotary;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TestUtil;

public class RevRotaryTest implements AutoCloseable {
    RevRotary revRotary;

    @BeforeEach
    void setup()
    {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

        revRotary = RevRotaryConstants.get();

        /* enable the robot */
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        /* delay ~100ms so the devices can start up and enable */
        Timer.delay(0.100);
    }

    @SuppressWarnings("PMD.SignatureDeclareThrowsException")
    @AfterEach
    void shutdown() throws Exception
    {
        close();
    }

    @Test
    void testGoToStow()
    {
        TestUtil.runTest(revRotary.setSetpoint(RevRotary.Setpoint.STOW), 3, revRotary);
        try {
            // Check to see if RevRotary subsystem is within tolerance of STOW setpoint
            assertTrue(revRotary.nearGoal(RevRotary.Setpoint.STOW.getSetpoint()));
        } catch (Exception e) {
            fail("Failed to run RevRotary Subsystem to STOW: " + e.getMessage());
        }
    }

    @Test
    void testGoToRaised()
    {
        TestUtil.runTest(revRotary.setSetpoint(RevRotary.Setpoint.RAISED), 3, revRotary);
        try {
            // Check to see if RevRotary subsystem is within tolerance of RAISED setpoint
            assertTrue(revRotary.nearGoal(RevRotary.Setpoint.RAISED.getSetpoint()));
        } catch (Exception e) {
            fail("Failed to run RevRotary Subsystem to RAISED: " + e.getMessage());
        }
    }

    @Test
    void testGoToGoalWithWait()
    {
        TestUtil.runTest(revRotary.setGoalCommandWithWait(RevRotary.Setpoint.STOW), 3, revRotary);
        try {
            // Check position to verify the subsystem is in tolerance of STOW setpoint
            assertTrue(revRotary.nearGoal(RevRotary.Setpoint.STOW.getSetpoint()));
        } catch (Exception e) {
            fail("Failed to run RevRotary Subsystem to STOW with wait: " + e.getMessage());
        }
    }

    @Test
    void testSetpointEnum()
    {
        // Test that all setpoints can be retrieved and have valid values
        try {
            RevRotary.Setpoint stow = RevRotary.Setpoint.STOW;
            RevRotary.Setpoint raised = RevRotary.Setpoint.RAISED;

            assertNotNull(stow.getSetpoint());
            assertNotNull(raised.getSetpoint());
            assertNotNull(stow.getTunableNumber());
            assertNotNull(raised.getTunableNumber());
        } catch (Exception e) {
            fail("Failed to retrieve setpoint values: " + e.getMessage());
        }
    }

    @Test
    void testNearGoalTolerance()
    {
        // Move to STOW position
        TestUtil.runTest(revRotary.setSetpoint(RevRotary.Setpoint.STOW), 3, revRotary);

        try {
            // Test that we're near the STOW goal
            assertTrue(revRotary.nearGoal(RevRotary.Setpoint.STOW.getSetpoint()));

            // Test that we're not near the RAISED goal when at STOW
            assertFalse(revRotary.nearGoal(RevRotary.Setpoint.RAISED.getSetpoint()));
        } catch (Exception e) {
            fail("Failed near goal tolerance test: " + e.getMessage());
        }
    }

    @Test
    void testGetVelocity()
    {
        // Start a movement
        TestUtil.runTest(revRotary.setSetpoint(RevRotary.Setpoint.RAISED), 0.5, revRotary);

        try {
            // Velocity should be accessible during movement
            assertNotNull(revRotary.getVelocity());
        } catch (Exception e) {
            fail("Failed get velocity test: " + e.getMessage());
        }
    }

    @Test
    void testMultipleSetpoints()
    {
        // Test moving between multiple setpoints
        TestUtil.runTest(revRotary.setSetpoint(RevRotary.Setpoint.STOW), 3, revRotary);
        assertTrue(revRotary.nearGoal(RevRotary.Setpoint.STOW.getSetpoint()));

        TestUtil.runTest(revRotary.setSetpoint(RevRotary.Setpoint.RAISED), 3, revRotary);
        assertTrue(revRotary.nearGoal(RevRotary.Setpoint.RAISED.getSetpoint()));

        TestUtil.runTest(revRotary.setSetpoint(RevRotary.Setpoint.STOW), 3, revRotary);
        assertTrue(revRotary.nearGoal(RevRotary.Setpoint.STOW.getSetpoint()));
    }

    @Test
    void testCommandNaming()
    {
        // Test that commands have descriptive names for debugging
        Command stowCmd = revRotary.setSetpoint(RevRotary.Setpoint.STOW);
        assertTrue(stowCmd.getName().contains("STOW"));

        Command raisedCmd = revRotary.setSetpoint(RevRotary.Setpoint.RAISED);
        assertTrue(raisedCmd.getName().contains("RAISED"));

        Command waitCmd = revRotary.setGoalCommandWithWait(RevRotary.Setpoint.STOW);
        assertTrue(waitCmd.getName().contains("wait"));
    }

    @Test
    void testPeriodicExecution()
    {
        // Test that periodic() can be called without errors
        try {
            revRotary.periodic();
            revRotary.periodic();
            revRotary.periodic();
        } catch (Exception e) {
            fail("Failed periodic execution test: " + e.getMessage());
        }
    }

    @Test
    void testRapidSetpointChanges()
    {
        // Test that rapid setpoint changes are handled correctly
        TestUtil.runTest(revRotary.setSetpoint(RevRotary.Setpoint.RAISED), 1, revRotary);
        TestUtil.runTest(revRotary.setSetpoint(RevRotary.Setpoint.STOW), 1, revRotary);
        TestUtil.runTest(revRotary.setSetpoint(RevRotary.Setpoint.RAISED), 3, revRotary);

        try {
            // Should end up at the final setpoint
            assertTrue(revRotary.nearGoal(RevRotary.Setpoint.RAISED.getSetpoint()));
        } catch (Exception e) {
            fail("Failed rapid setpoint changes test: " + e.getMessage());
        }
    }

    @Override
    public void close()
    {
        revRotary.close();
    }
}
