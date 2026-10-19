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

/**
 * How much the pose estimator should trust one accepted vision pose: standard deviations for x, y
 * (meters) and heading (radians). Smaller means trusted more.
 *
 * <p>This is Spectrum 3847's trust model (their {@code TrustModel} in {@code
 * org.spectrum3847.vision.trust}), adapted to our types. The base grows with distance squared over
 * the number of tags, since the error of a tag's position estimate grows roughly with distance
 * squared and independent tags average it down. It is then multiplied by everything else that makes
 * a measurement worse: tags near the image edge, a high reprojection error, a low decision margin,
 * single-tag ambiguity, and the robot moving or turning.
 *
 * <p>Every factor is a public field so it can be tuned.
 */
public class TrustModel {
    /** x/y standard deviation for one tag at 1 m, meters. */
    public double xyPerMeterSquared = 0.02;

    /** Heading standard deviation for multi-tag at 1 m, radians. */
    public double thetaPerMeterSquared = 0.06;

    /** Floors, so a close tag never claims centimeter accuracy. */
    public double minXy = 0.02;

    public double minTheta = Math.toRadians(1.5);

    /** Heading standard deviation for solves whose heading is not worth using: ignored. */
    public double ignoredTheta = Double.POSITIVE_INFINITY;

    /** Single-tag solves: heading ignored (it is the least reliable part of one tag). */
    public boolean ignoreSingleTagHeading = true;

    /** Edge falloff: below this many pixels from the edge, trust drops... */
    public double edgeSoftPx = 60;

    /** ...up to this factor at the very edge. */
    public double edgeMaxFactor = 3.0;

    /** Reprojection: factor 1 + error / this (pixels). */
    public double reprojScalePx = 1.5;

    /** Decision margin: below this, factor softMargin / margin. */
    public double softDecisionMargin = 40;

    /** Ambiguity (single tag): factor 1 + this * ambiguity. */
    public double ambiguityGain = 4.0;

    /** Motion: factor 1 + speed * this (per m/s)... */
    public double perMeterPerSecond = 0.15;

    /** ...+ |turn rate| * this (per rad/s). */
    public double perRadianPerSecond = 0.4;

    /**
     * What is known about one vision pose. Use {@code NaN} for anything unavailable and that factor
     * is skipped.
     *
     * @param tagCount tags that contributed to the pose
     * @param averageDistanceMeters mean camera-to-tag distance, or NaN if unknown (treated as 3 m)
     * @param ambiguity single-tag pose ambiguity in [0, 1], or NaN/0 when not applicable
     * @param minEdgePx smallest distance from any tag corner to the image border, in pixels
     * @param reprojErrPx mean reprojection error of the solve, in pixels
     * @param minDecisionMargin lowest AprilTag decision margin among the tags
     */
    public record Observation(
            int tagCount,
            double averageDistanceMeters,
            double ambiguity,
            double minEdgePx,
            double reprojErrPx,
            double minDecisionMargin) {}

    /** The robot's motion when the frame was captured. */
    public record Motion(double speedMetersPerSecond, double yawRateRadiansPerSecond) {}

    /** Standard deviations for a vision pose: meters for x and y, radians for heading. */
    public record StdDevs(double xy, double theta) {}

    /** Our defaults (the fields above). */
    public static TrustModel recommended() {
        return new TrustModel();
    }

    /**
     * The standard deviations for an observation.
     *
     * @param cameraFactor a camera's own multiplier: a poorly calibrated or wobbly camera is
     *     trusted less
     */
    public StdDevs stdDevs(Observation o, Motion motion, double cameraFactor) {
        int n = Math.max(1, o.tagCount());
        double d = Double.isNaN(o.averageDistanceMeters()) ? 3.0 : o.averageDistanceMeters();
        double f = cameraFactor * qualityFactor(o) * motionFactor(motion);
        double xy = Math.max(minXy, xyPerMeterSquared * d * d / n) * f;
        double theta = Math.max(minTheta, thetaPerMeterSquared * d * d / n) * f;
        if (o.tagCount() <= 1 && ignoreSingleTagHeading) {
            theta = ignoredTheta;
        }
        return new StdDevs(xy, theta);
    }

    /** Edge, reprojection, decision margin and ambiguity together (1 = perfect). */
    public double qualityFactor(Observation o) {
        double f = 1;
        if (!Double.isNaN(o.minEdgePx()) && o.minEdgePx() < edgeSoftPx) {
            f *= 1 + (edgeMaxFactor - 1) * (edgeSoftPx - Math.max(0, o.minEdgePx())) / edgeSoftPx;
        }
        if (!Double.isNaN(o.reprojErrPx())) {
            f *= 1 + o.reprojErrPx() / reprojScalePx;
        }
        if (!Double.isNaN(o.minDecisionMargin()) && o.minDecisionMargin() < softDecisionMargin) {
            f *= softDecisionMargin / Math.max(5, o.minDecisionMargin());
        }
        if (o.tagCount() <= 1 && o.ambiguity() > 0) {
            f *= 1 + ambiguityGain * o.ambiguity();
        }
        return f;
    }

    /** Robot speed and turn rate: 1 when standing still, larger the faster it moves or turns. */
    public double motionFactor(Motion motion) {
        return 1
                + Math.abs(motion.speedMetersPerSecond()) * perMeterPerSecond
                + Math.abs(motion.yawRateRadiansPerSecond()) * perRadianPerSecond;
    }
}
