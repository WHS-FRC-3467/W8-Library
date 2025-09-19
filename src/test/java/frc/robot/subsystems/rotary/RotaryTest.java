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
import static org.junit.jupiter.api.Assertions.fail;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.TestUtil;

public class RotaryTest implements AutoCloseable {
    RotarySubsystem rotary;

    @BeforeEach // this method will run before each test
    void setup()
    {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

        rotary = new RotarySubsystem(RotarySubsystemConstants.getSim());

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
        TestUtil.runTest(rotary.setSetpoint(RotarySubsystem.Setpoint.RAISED), 2, rotary);
        try {
            // Check to see if Rotary subsystem is within tolerance of RAISED setpoint.
            assertTrue(rotary.nearGoal(RotarySubsystem.Setpoint.RAISED.getSetpoint()));
        } catch (Exception e) {
            fail("Failed to run Rotary Subsystem to RAISED: " + e.getMessage());
        }
    }

    @Test // marks this method as a test
    void goToGoalWithWait()
    {
        TestUtil.runTest(rotary.setGoalCommandWithWait(RotarySubsystem.Setpoint.STOW), 2, rotary);
        try {
            // Check position to check if the subsystem is actually in tolerance of STOW setpoint.
            assertTrue(rotary.nearGoal(RotarySubsystem.Setpoint.STOW.getSetpoint()));
        } catch (Exception e) {
            fail("Failed to run Rotary Subsystem to STOW: " + e.getMessage());
        }
    }

    @Override
    public void close()
    {
        rotary.close();
    }
}
