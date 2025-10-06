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

package frc.robot.subsystems.linear;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.TestUtil;

public class LinearTest implements AutoCloseable {
    static final double DELTA = 1e-2; // acceptable deviation range
    Linear linear;

    @BeforeEach // this method will run before each test
    void setup()
    {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

        linear = new Linear(LinearConstants.getSim());

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

    @Test // marks this method as a test
    void home()
    {
        TestUtil.runTest(linear.homeCommand(), 0.1, linear);
        try {
            // Check position to check if it is homed, and within tolerance of STOW setpoint.
            assertTrue(linear.nearGoal(Linear.Setpoint.STOW.getSetpoint()));
        } catch (Exception e) {
            fail("Failed to home Linear Subsystem: " + e.getMessage());
        }
    }

    @Test
    void goToGoal()
    {
        TestUtil.runTest(linear.setGoal(Linear.Setpoint.RAISED), 2, linear);
        try {
            // Check to see if linear subsystem is within tolerance of RAISED setpoint.
            assertTrue(linear.nearGoal(Linear.Setpoint.RAISED.getSetpoint()));
        } catch (Exception e) {
            fail("Failed to run Linear Subsystem to RAISED: " + e.getMessage());
        }
    }

    @Override
    public void close()
    {
        linear.close();
    }
}
