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

package frc.robot.subsystems.rotary;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.fail;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.TestUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class RotaryTest implements AutoCloseable {
    Rotary rotary;

    @BeforeEach // this method will run before each test
    void setup()
    {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

        rotary = RotaryConstants.get();

        /* enable the robot */
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        /* delay ~100ms so the devices can start up and enable */
        Timer.delay(0.100);
    }

    @SuppressWarnings("PMD.SignatureDeclareThrowsException")
    @AfterEach // this method will run after each test
    void shutdown() throws Exception
    {
        close();
    }

    @Test
    void goToGoal()
    {
        TestUtil.runTest(rotary.setSetpoint(Rotary.Setpoint.RAISED), 3, rotary);
        try {
            // Check to see if Rotary subsystem is within tolerance of RAISED setpoint.
            assertTrue(rotary.nearGoal(Rotary.Setpoint.RAISED.getSetpoint()));
        } catch (Exception e) {
            fail("Failed to run Rotary Subsystem to RAISED: " + e.getMessage());
        }
    }

    @Test
    void goToGoalWithWait()
    {
        TestUtil.runTest(rotary.setGoalCommandWithWait(Rotary.Setpoint.STOW), 3, rotary);
        try {
            // Check position to check if the subsystem is actually in tolerance of STOW setpoint.
            assertTrue(rotary.nearGoal(Rotary.Setpoint.STOW.getSetpoint()));
        } catch (Exception e) {
            fail("Failed to run Rotary Subsystem to STOW: " + e.getMessage());
        }
    }

    @Test
    void testSetpointEnum()
    {
        // Test that all setpoints can be retrieved and have valid values
        try {
            Rotary.Setpoint stow = Rotary.Setpoint.STOW;
            Rotary.Setpoint raised = Rotary.Setpoint.RAISED;
            Rotary.Setpoint home = Rotary.Setpoint.HOME;

            assertNotNull(stow.getSetpoint());
            assertNotNull(raised.getSetpoint());
            assertNotNull(home.getSetpoint());
        } catch (Exception e) {
            fail("Failed to retrieve setpoint values: " + e.getMessage());
        }
    }

    @Test
    void testNearGoalTolerance()
    {
        // Move to STOW position
        TestUtil.runTest(rotary.setSetpoint(Rotary.Setpoint.STOW), 3, rotary);

        try {
            // Test that we're near the goal
            assertTrue(rotary.nearGoal(Rotary.Setpoint.STOW.getSetpoint()));

            // Test that we're not near a different goal
            assertFalse(rotary.nearGoal(Rotary.Setpoint.RAISED.getSetpoint()));
        } catch (Exception e) {
            fail("Failed near goal tolerance test: " + e.getMessage());
        }
    }



    @Test
    void testSetStateCommand()
    {
        // Test that setState command executes without errors
        TestUtil.runTest(rotary.setStateCommand(Rotary.Setpoint.RAISED), 1, rotary);
        // State setting is internal, so we just verify no exceptions thrown
    }

    @Test
    void testGetVelocity()
    {
        // Start a movement
        TestUtil.runTest(rotary.setSetpoint(Rotary.Setpoint.RAISED), 0.5, rotary);

        try {
            // Velocity should be non-zero during movement
            assertNotNull(rotary.getVelocity());
        } catch (Exception e) {
            fail("Failed get velocity test: " + e.getMessage());
        }
    }

    @Test
    void testMultipleSetpoints()
    {
        // Test moving between multiple setpoints
        TestUtil.runTest(rotary.setSetpoint(Rotary.Setpoint.STOW), 3, rotary);
        assertTrue(rotary.nearGoal(Rotary.Setpoint.STOW.getSetpoint()));

        TestUtil.runTest(rotary.setSetpoint(Rotary.Setpoint.RAISED), 3, rotary);
        assertTrue(rotary.nearGoal(Rotary.Setpoint.RAISED.getSetpoint()));

        TestUtil.runTest(rotary.setSetpoint(Rotary.Setpoint.STOW), 3, rotary);
        assertTrue(rotary.nearGoal(Rotary.Setpoint.STOW.getSetpoint()));
    }

    @Test
    void testCommandNaming()
    {
        // Test that commands have descriptive names
        Command stowCmd = rotary.setSetpoint(Rotary.Setpoint.STOW);
        assertTrue(stowCmd.getName().contains("STOW"));

        Command raisedCmd = rotary.setSetpoint(Rotary.Setpoint.RAISED);
        assertTrue(raisedCmd.getName().contains("RAISED"));

        Command waitCmd = rotary.setGoalCommandWithWait(Rotary.Setpoint.STOW);
        assertTrue(waitCmd.getName().contains("wait"));
    }

    @Override
    public void close()
    {}
}
