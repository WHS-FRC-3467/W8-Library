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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;

import frc.lib.devices.VisionSource;
import frc.lib.devices.VisionSource.CameraProperties;
import frc.lib.io.vision.VisionIO;
import frc.lib.io.vision.VisionIOC2;
import frc.lib.io.vision.VisionIOC2Sim;
import frc.lib.io.vision.VisionIOPhotonVisionSim;
import frc.robot.Constants;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.RobotState;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;

import java.util.List;

/**
 * Configuration constants for the vision subsystem.
 *
 * <p>Contains camera calibration data, mounting positions, and factory methods for creating vision
 * cameras. Each camera has:
 *
 * <ul>
 *   <li>Extrinsics: Physical mounting transform (position and orientation on robot)
 *   <li>Intrinsics: Camera matrix and distortion coefficients from calibration
 *   <li>Performance: Resolution, FPS, latency, and standard deviation factors
 * </ul>
 *
 * <p>Camera intrinsics should be recalibrated when cameras are changed or remounted. Use
 * PhotonVision's calibration tool to generate new intrinsic matrices.
 */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class VisionConstants {
    public static final String NAME = "Vision";

    public static final AprilTagFieldLayout fieldLayout = AprilTagLayoutType.NO_TRENCH.getLayout();

    /** Where simulated camera results come from. */
    public enum SimBackend {
        /**
         * The real C2 code in simulation mode ({@code python src/main.py --sim} in C2, on this
         * computer). Exercises the exact real-robot path: C2's detector, NT topics, timestamps.
         */
        C2,
        /** PhotonVision's built-in simulator. No external process needed. */
        PHOTONVISION
    }

    public static final SimBackend SIM_BACKEND = SimBackend.C2;

    // Extrinsics
    public static final String FRONT_LEFT_NAME = "front_left";
    public static final String LEFT_NAME = "left";
    public static final String RIGHT_NAME = "right";
    public static final String FRONT_RIGHT_NAME = "front_right";

    // Which slice of C2's combined frame each camera is.
    private static final int FRONT_LEFT_C2_SLICE = 3;
    private static final int LEFT_C2_SLICE = 1;
    private static final int RIGHT_C2_SLICE = 0;
    private static final int FRONT_RIGHT_C2_SLICE = 2;

    public static final Transform3d FRONT_LEFT_TRANSFORM =
            new Transform3d(
                    Units.inchesToMeters(13.106),
                    Units.inchesToMeters(5.25),
                    Units.inchesToMeters(12.681),
                    new Rotation3d(
                            Units.degreesToRadians(0.0),
                            Units.degreesToRadians(-25.0),
                            Units.degreesToRadians(0.0)));
    public static final Transform3d LEFT_TRANSFORM =
            new Transform3d(
                    Units.inchesToMeters(12.749),
                    Units.inchesToMeters(13.293334),
                    Units.inchesToMeters(16.758),
                    new Rotation3d(
                            0.0, Units.degreesToRadians(-20.0), Units.degreesToRadians(90.0)));
    public static final Transform3d RIGHT_TRANSFORM =
            new Transform3d(
                    Units.inchesToMeters(12.749),
                    Units.inchesToMeters(-13.299),
                    Units.inchesToMeters(16.758),
                    new Rotation3d(
                            0.0, Units.degreesToRadians(-20.0), Units.degreesToRadians(-90.00)));
    public static final Transform3d FRONT_RIGHT_TRANSFORM =
            new Transform3d(
                    Units.inchesToMeters(13.106),
                    Units.inchesToMeters(-5.25),
                    Units.inchesToMeters(12.681),
                    new Rotation3d(
                            Units.degreesToRadians(0.0),
                            Units.degreesToRadians(-25.0),
                            Units.degreesToRadians(0.0)));

    // Intrinsics
    // ThriftyCam Default Calibrations
    // FRONT_LEFT = 001
    public static final Matrix<N3, N3> FRONT_LEFT_MATRIX =
            MatBuilder.fill(
                    Nat.N3(),
                    Nat.N3(),
                    1380.531127613026,
                    0.0,
                    765.2809506558299,
                    0.0,
                    1381.5640806035888,
                    626.9982046388377,
                    0.0,
                    0.0,
                    1.0);

    public static final Vector<N8> FRONT_LEFT_DIST_COEFFS =
            VecBuilder.fill(
                    -0.029800389169817133,
                    0.011335486937787855,
                    -.0007758929871136083,
                    -.000979966348825576,
                    0.004322943914551887,
                    .000977625446561076,
                    -0.001486288707185138,
                    .0003931042262115059);

    // LEFT = 002
    public static final Matrix<N3, N3> LEFT_MATRIX =
            MatBuilder.fill(
                    Nat.N3(),
                    Nat.N3(),
                    2000.0032334309244,
                    0,
                    813.8184817447087,
                    0,
                    1997.6334218966347,
                    688.2406381921382,
                    0,
                    0,
                    1);

    public static final Vector<N8> LEFT_DIST_COEFFS =
            VecBuilder.fill(
                    0.06485476013833127,
                    0.28421734688292644,
                    -0.000861882706123709,
                    0.0005123699898998346,
                    -1.0948497906268129,
                    -0.005476103701277743,
                    -0.0075079750325369385,
                    0.044951833493942085);

    // RIGHT = 004
    public static final Matrix<N3, N3> RIGHT_MATRIX =
            MatBuilder.fill(
                    Nat.N3(),
                    Nat.N3(),
                    1999.9919360266583,
                    0,
                    759.8976902097196,
                    0,
                    2000.756601602442,
                    610.5233343357824,
                    0,
                    0,
                    1);

    public static final Vector<N8> RIGHT_DIST_COEFFS =
            VecBuilder.fill(
                    0.10368101039813213,
                    -0.08495511830294374,
                    -0.001968724475841204,
                    -0.0005170525371013939,
                    -0.24440475107943974,
                    -0.005569982774319356,
                    0.0038802482316867243,
                    0.010015510605366526);

    // FRONT_RIGHT = 003
    public static final Matrix<N3, N3> FRONT_RIGHT_MATRIX =
            MatBuilder.fill(
                    Nat.N3(),
                    Nat.N3(),
                    1994.3346993932607,
                    0,
                    772.1194963312653,
                    0,
                    1990.5668437725847,
                    559.9647344141187,
                    0,
                    0,
                    1);

    public static final Vector<N8> FRONT_RIGHT_DIST_COEFFS =
            VecBuilder.fill(
                    0.10622095692661696,
                    -0.05338512237989565,
                    -0.0030328707104387877,
                    0.0003234715047390942,
                    -0.3410348671624887,
                    -0.005797616380180348,
                    0.0033869449584776738,
                    0.013942594262017396);

    public static final int FRONT_LEFT_RESOLUTION_WIDTH = 1600;
    public static final int FRONT_LEFT_RESOLUTION_HEIGHT = 1304;
    public static final int LEFT_RESOLUTION_WIDTH = 1600;
    public static final int LEFT_RESOLUTION_HEIGHT = 1304;
    public static final int RIGHT_RESOLUTION_WIDTH = 1600;
    public static final int RIGHT_RESOLUTION_HEIGHT = 1304;
    public static final int FRONT_RIGHT_RESOLUTION_WIDTH = 1600;
    public static final int FRONT_RIGHT_RESOLUTION_HEIGHT = 1304;

    public static final Angle FRONT_LEFT_FOV = Degrees.of(80); // from Thrifty docs
    public static final Angle LEFT_FOV = Degrees.of(55);
    public static final Angle RIGHT_FOV = Degrees.of(55);
    public static final Angle FRONT_RIGHT_FOV = Degrees.of(55);

    // Performance
    public static final double FRONT_LEFT_FPS = 22;
    public static final double LEFT_FPS = 22;
    public static final double RIGHT_FPS = 22;
    public static final double FRONT_RIGHT_FPS = 22;

    public static final double FRONT_LEFT_STDDEV_FACTOR = 1.0;
    public static final double LEFT_STDDEV_FACTOR = 1.0;
    public static final double RIGHT_STDDEV_FACTOR = 1.0;
    public static final double FRONT_RIGHT_STDDEV_FACTOR = 1.0;

    // Exposure 5 ms, USB 5 ms, detection 15 ms, scheduling 5 ms
    public static final Time FRONT_LEFT_LATENCY = Milliseconds.of(30);
    public static final Time LEFT_LATENCY = Milliseconds.of(30);
    public static final Time RIGHT_LATENCY = Milliseconds.of(30);
    public static final Time FRONT_RIGHT_LATENCY = Milliseconds.of(30);

    public static final Time FRONT_LEFT_LATENCY_STDDEV = Milliseconds.of(5);
    public static final Time LEFT_LATENCY_STDDEV = Milliseconds.of(5);
    public static final Time RIGHT_LATENCY_STDDEV = Milliseconds.of(5);
    public static final Time FRONT_RIGHT_LATENCY_STDDEV = Milliseconds.of(5);

    public static final CameraProperties FRONT_LEFT =
            new CameraProperties(
                    FRONT_LEFT_NAME,
                    FRONT_LEFT_TRANSFORM,
                    FRONT_LEFT_MATRIX,
                    FRONT_LEFT_DIST_COEFFS,
                    FRONT_LEFT_RESOLUTION_WIDTH,
                    FRONT_LEFT_RESOLUTION_HEIGHT,
                    FRONT_LEFT_STDDEV_FACTOR,
                    FRONT_LEFT_FOV,
                    FRONT_LEFT_FPS,
                    FRONT_LEFT_LATENCY,
                    FRONT_LEFT_LATENCY_STDDEV);

    public static final CameraProperties LEFT =
            new CameraProperties(
                    LEFT_NAME,
                    LEFT_TRANSFORM,
                    LEFT_MATRIX,
                    LEFT_DIST_COEFFS,
                    LEFT_RESOLUTION_WIDTH,
                    LEFT_RESOLUTION_HEIGHT,
                    LEFT_STDDEV_FACTOR,
                    LEFT_FOV,
                    LEFT_FPS,
                    LEFT_LATENCY,
                    LEFT_LATENCY_STDDEV);

    public static final CameraProperties RIGHT =
            new CameraProperties(
                    RIGHT_NAME,
                    RIGHT_TRANSFORM,
                    RIGHT_MATRIX,
                    RIGHT_DIST_COEFFS,
                    RIGHT_RESOLUTION_WIDTH,
                    RIGHT_RESOLUTION_HEIGHT,
                    RIGHT_STDDEV_FACTOR,
                    RIGHT_FOV,
                    RIGHT_FPS,
                    RIGHT_LATENCY,
                    RIGHT_LATENCY_STDDEV);

    public static final CameraProperties FRONT_RIGHT =
            new CameraProperties(
                    FRONT_RIGHT_NAME,
                    FRONT_RIGHT_TRANSFORM,
                    FRONT_RIGHT_MATRIX,
                    FRONT_RIGHT_DIST_COEFFS,
                    FRONT_RIGHT_RESOLUTION_WIDTH,
                    FRONT_RIGHT_RESOLUTION_HEIGHT,
                    FRONT_RIGHT_STDDEV_FACTOR,
                    FRONT_RIGHT_FOV,
                    FRONT_RIGHT_FPS,
                    FRONT_RIGHT_LATENCY,
                    FRONT_RIGHT_LATENCY_STDDEV);

    /** Every camera, on one C2 coprocessor (and on PhotonVision in PhotonVision sim). */
    public static final List<CameraProperties> CAMERAS =
            List.of(FRONT_LEFT, LEFT, RIGHT, FRONT_RIGHT);

    /** The C2 device: which slice of its combined frame each of {@link #CAMERAS} is. */
    private static final VisionIOC2.C2Config C2_CONFIG =
            VisionIOC2.C2Config.defaults(
                    FRONT_LEFT_RESOLUTION_WIDTH,
                    FRONT_LEFT_RESOLUTION_HEIGHT,
                    List.of(
                            FRONT_LEFT_C2_SLICE,
                            LEFT_C2_SLICE,
                            RIGHT_C2_SLICE,
                            FRONT_RIGHT_C2_SLICE));

    /**
     * The single vision source's name. The same in every mode, so its logged inputs are under the
     * same key whichever IO recorded them, and replay reads them without any configuration.
     */
    private static final String SOURCE_NAME = "main";

    /**
     * Creates the VisionSubsystem with the vision source for the current robot mode (real, sim, or
     * replay).
     */
    public static void create() {
        VisionIO io =
                switch (Constants.currentMode) {
                    case REAL -> new VisionIOC2(C2_CONFIG);
                    case SIM ->
                            switch (SIM_BACKEND) {
                                // Same IO as the real robot; C2 (in sim mode) renders the cameras
                                // from the true pose.
                                case C2 ->
                                        new VisionIOC2Sim(
                                                C2_CONFIG,
                                                CAMERAS,
                                                RobotState.getInstance()::getOdometrySamplesAfter);
                                case PHOTONVISION ->
                                        new VisionIOPhotonVisionSim(
                                                CAMERAS,
                                                () -> RobotState.getInstance().getOdometryPose(),
                                                fieldLayout);
                            };
                    // The log says which packet type each result is (C2 or PhotonVision).
                    case REPLAY -> new VisionIO() {};
                };
        new VisionSubsystem(new VisionSource(SOURCE_NAME, CAMERAS, io, fieldLayout));
    }
}
