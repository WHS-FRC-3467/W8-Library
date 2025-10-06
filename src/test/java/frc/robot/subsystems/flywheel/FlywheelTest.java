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
package frc.robot.subsystems.flywheel;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.TestUtil;
import edu.wpi.first.wpilibj.Timer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static edu.wpi.first.units.Units.Amps;
import static org.junit.jupiter.api.Assertions.fail;

class FlywheelTest implements AutoCloseable {
    static final double DELTA = 1e-2; // acceptable deviation range
    Flywheel flywheel;

    @BeforeEach // this method will run before each test
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

        flywheel = new Flywheel(FlywheelConstants.getSim());

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

    @Test // marks this method as a test
    void shoot() {
        TestUtil.runTest(flywheel.shootAmps(), 0.5, flywheel);
        try {
            assertEquals(30, flywheel.getTorqueCurrent().in(Amps), DELTA);
        } catch (Exception e) {
            fail("Failed to run the Flywheel at 30 amps: " + e.getMessage());
        }
    }

    @Override
    public void close() {
        flywheel.close();
    }
}
