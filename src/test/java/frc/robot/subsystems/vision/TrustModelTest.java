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

package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.vision.TrustModel.Motion;
import frc.robot.subsystems.vision.TrustModel.Observation;
import frc.robot.subsystems.vision.TrustModel.StdDevs;

import org.junit.jupiter.api.Test;

class TrustModelTest {
    private static final double NAN = Double.NaN;
    private static final Motion STILL = new Motion(0.0, 0.0);

    private final TrustModel model = TrustModel.recommended();

    /** A clean observation: nothing known beyond tag count and distance. */
    private static Observation clean(int tags, double distanceMeters) {
        return new Observation(tags, distanceMeters, 0.0, NAN, NAN, NAN);
    }

    private StdDevs stdDevs(Observation observation) {
        return model.stdDevs(observation, STILL, 1.0);
    }

    // ---- base: distance squared over tag count ----

    @Test
    void oneTagAtThreeMetersIsTwoCentimetersTimesNine() {
        assertEquals(0.18, stdDevs(clean(1, 3.0)).xy(), 1e-12);
    }

    @Test
    void errorGrowsWithDistanceSquared() {
        assertEquals(4.0, stdDevs(clean(1, 4.0)).xy() / stdDevs(clean(1, 2.0)).xy(), 1e-12);
    }

    @Test
    void independentTagsAverageTheErrorDown() {
        assertEquals(0.09, stdDevs(clean(2, 3.0)).xy(), 1e-12);
        assertEquals(stdDevs(clean(1, 3.0)).xy() / 4.0, stdDevs(clean(4, 3.0)).xy(), 1e-12);
    }

    @Test
    void headingIsAMultiTagQuantity() {
        // 0.06 * d^2 / n for two tags at 3 m
        assertEquals(0.27, stdDevs(clean(2, 3.0)).theta(), 1e-12);
    }

    @Test
    void aSingleTagHeadingIsIgnoredAsAnInfiniteStdDev() {
        StdDevs single = stdDevs(clean(1, 2.0));

        // The estimator leaves infinite-variance axes out of the update
        assertTrue(Double.isInfinite(single.theta()));
        assertTrue(Double.isFinite(single.xy()));
    }

    @Test
    void closeTagsNeverClaimBetterThanTheFloors() {
        StdDevs close = stdDevs(clean(4, 0.5)); // formula gives 1.25 cm and 0.2 degrees

        assertEquals(0.02, close.xy(), 1e-12);
        assertEquals(Math.toRadians(1.5), close.theta(), 1e-12);
    }

    @Test
    void anUnknownDistanceIsTreatedAsThreeMeters() {
        assertEquals(stdDevs(clean(1, 3.0)).xy(), stdDevs(clean(1, NAN)).xy(), 1e-12);
    }

    // ---- multipliers ----

    @Test
    void cameraFactorScalesBothXyAndHeading() {
        Observation observation = clean(2, 3.0);
        StdDevs base = model.stdDevs(observation, STILL, 1.0);
        StdDevs doubled = model.stdDevs(observation, STILL, 2.0);

        assertEquals(2.0 * base.xy(), doubled.xy(), 1e-12);
        assertEquals(2.0 * base.theta(), doubled.theta(), 1e-12);
    }

    @Test
    void movingAndTurningMakeAMeasurementWorse() {
        // 1 + 2 m/s * 0.15 + 1 rad/s * 0.4
        assertEquals(1.7, model.motionFactor(new Motion(2.0, 1.0)), 1e-12);
        assertEquals(1.0, model.motionFactor(STILL), 1e-12);
        // Direction does not matter
        assertEquals(1.7, model.motionFactor(new Motion(-2.0, -1.0)), 1e-12);

        assertEquals(
                1.7 * stdDevs(clean(1, 3.0)).xy(),
                model.stdDevs(clean(1, 3.0), new Motion(2.0, 1.0), 1.0).xy(),
                1e-12);
    }

    @Test
    void anUnknownMeasurementContributesNothing() {
        assertEquals(1.0, model.qualityFactor(clean(2, 3.0)), 1e-12);
    }

    @Test
    void tagsNearTheImageEdgeAreTrustedLess() {
        // Falls from 1 at 60 px to 3 at the very edge
        assertEquals(1.0, model.qualityFactor(new Observation(2, 3.0, 0.0, 60.0, NAN, NAN)), 1e-12);
        assertEquals(
                1.0, model.qualityFactor(new Observation(2, 3.0, 0.0, 200.0, NAN, NAN)), 1e-12);
        assertEquals(2.0, model.qualityFactor(new Observation(2, 3.0, 0.0, 30.0, NAN, NAN)), 1e-12);
        assertEquals(3.0, model.qualityFactor(new Observation(2, 3.0, 0.0, 0.0, NAN, NAN)), 1e-12);
    }

    @Test
    void aHighReprojectionErrorIsTrustedLess() {
        // 1 + 3 px / 1.5 px
        assertEquals(3.0, model.qualityFactor(new Observation(2, 3.0, 0.0, NAN, 3.0, NAN)), 1e-12);
    }

    @Test
    void aLowDecisionMarginIsTrustedLess() {
        assertEquals(2.0, model.qualityFactor(new Observation(2, 3.0, 0.0, NAN, NAN, 20.0)), 1e-12);
        assertEquals(1.0, model.qualityFactor(new Observation(2, 3.0, 0.0, NAN, NAN, 60.0)), 1e-12);
    }

    @Test
    void ambiguityOnlyCountsForASingleTag() {
        // 1 + 4 * 0.5
        assertEquals(3.0, model.qualityFactor(new Observation(1, 3.0, 0.5, NAN, NAN, NAN)), 1e-12);
        assertEquals(1.0, model.qualityFactor(new Observation(2, 3.0, 0.5, NAN, NAN, NAN)), 1e-12);
    }

    @Test
    void qualityFactorsMultiply() {
        // edge 2.0 * reprojection 3.0 * ambiguity 3.0
        Observation observation = new Observation(1, 3.0, 0.5, 30.0, 3.0, NAN);

        assertEquals(18.0, model.qualityFactor(observation), 1e-12);
    }

    @Test
    void everythingComposesIntoTheStdDev() {
        Observation observation = new Observation(2, 3.0, 0.0, 30.0, NAN, NAN);
        Motion motion = new Motion(2.0, 1.0);

        // base 0.09 * camera 1.5 * quality 2.0 * motion 1.7
        assertEquals(0.09 * 1.5 * 2.0 * 1.7, model.stdDevs(observation, motion, 1.5).xy(), 1e-12);
    }
}
