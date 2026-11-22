/*
 * Copyright (C) 2026 Windham Windup
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

package frc.robot.subsystems.superstructure;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.intake.IntakeLinearConstants;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.intake.IntakeRollerConstants;
import frc.robot.subsystems.intake.IntakeSlide;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.FlywheelConstants;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.HoodConstants;
import frc.robot.subsystems.superstructure.Superstructure.Goal;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.tower.TowerConstants;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;

/** Runs the whole superstructure against the simulated mechanisms. */
class SuperstructureTest {
    /** Five seconds at the 20 ms loop period. */
    private static final int MAX_LOOPS = 250;

    private static final long LOOP_PERIOD_MS = 20;

    private static final Scheduler scheduler = Scheduler.getDefault();

    private static IntakeSlide slide;
    private static IntakeRoller roller;
    private static Indexer indexer;
    private static Tower tower;
    private static Flywheel flywheel;
    private static Hood hood;
    private static Superstructure superstructure;

    @BeforeAll
    static void setUp() {
        assertTrue(HAL.initialize());
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        slide = new IntakeSlide(IntakeLinearConstants.getMechanism());
        roller = new IntakeRoller(IntakeRollerConstants.getMechanism());
        indexer = IndexerConstants.get();
        tower = TowerConstants.get();
        flywheel = new Flywheel(FlywheelConstants.get());
        hood = new Hood(HoodConstants.get());
        superstructure = new Superstructure(slide, roller, indexer, tower, flywheel, hood);
    }

    @AfterAll
    static void tearDown() {
        scheduler.cancelAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        slide.close();
        roller.close();
        indexer.close();
        tower.close();
        flywheel.close();
        hood.close();
    }

    /**
     * Runs the scheduler at the normal loop rate until the condition is true. Returns false if it
     * never became true.
     *
     * <p>This waits in real time on purpose. The actuator sims and the simulated motor controllers
     * advance with the clock, so looping as fast as possible would give them no time to move.
     */
    private static boolean runUntil(BooleanSupplier condition) {
        for (int i = 0; i < MAX_LOOPS; i++) {
            scheduler.run();
            if (condition.getAsBoolean()) {
                return true;
            }
            try {
                Thread.sleep(LOOP_PERIOD_MS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return false;
            }
        }
        return false;
    }

    @Test
    void fullCycle() {
        scheduler.run();
        assertEquals(Goal.IDLE, superstructure.getState());

        // Intake: the slide goes out, and the roller only starts once it is clear of the robot.
        scheduler.schedule(superstructure.intake());
        assertTrue(runUntil(() -> superstructure.getState() == Goal.INTAKE));
        assertFalse(slide.isRollerSafe.getAsBoolean());
        assertTrue(runUntil(slide.isExtended), "slide never extended");
        assertTrue(slide.isRollerSafe.getAsBoolean());

        // Score: the robot aims first, then moves to SHOOT by itself once the shooter is ready.
        scheduler.schedule(superstructure.score());
        assertTrue(runUntil(() -> superstructure.getState() == Goal.AIM));
        assertFalse(superstructure.isShooting.getAsBoolean());
        assertTrue(
                runUntil(superstructure.isShooting),
                () -> "never reached SHOOT: flywheel " + flywheel.getVelocity() + " atSpeed "
                        + flywheel.atShootSpeed.getAsBoolean() + ", hood " + hood.getAngle() + " atAngle "
                        + hood.atShootAngle.getAsBoolean());
        assertTrue(superstructure.readyToShoot.getAsBoolean());

        // Stow: back to IDLE, and the shooter stops reporting ready.
        scheduler.schedule(superstructure.stow());
        assertTrue(runUntil(() -> superstructure.getState() == Goal.IDLE));
        assertTrue(runUntil(slide.isRetracted), "slide never retracted");
        // The request takes a loop or two to pass down to the flywheel and back up through the
        // triggers, so give it a few loops rather than checking on the very next one.
        assertTrue(runUntil(() -> !superstructure.readyToShoot.getAsBoolean()), "shooter still reports ready");
    }
}
