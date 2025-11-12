// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.posestimator.visionprocessors;

import static edu.wpi.first.units.Units.Meters;
import java.util.Optional;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.io.vision.VisionIO.VisionObservation;
import frc.lib.posestimator.PoseEstimator.VisionProcessor;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;
import lombok.experimental.Accessors;

/**
 * A {@link VisionProcessor} implementation that estimates the robot's pose using multi-tag camera
 * pose data produced directly by a coprocessor.
 *
 * <p>
 * This processor assumes that the vision coprocessor provides a precomputed camera pose based on
 * multiple AprilTag detections, and converts that into a field-relative robot pose using the known
 * camera-to-robot transform.
 */
@NoArgsConstructor
@Accessors(fluent = true)
public class MultiTagOnCoproc implements VisionProcessor {

    /** Default scaling factor for linear standard deviation (distance-based uncertainty). */
    private static final double DEFAULT_LINEAR_STDDEV_FACTOR = 0.4;

    /** Default scaling factor for angular standard deviation (orientation-based uncertainty). */
    private static final double DEFAULT_ANGULAR_STDDEV_FACTOR = 0.4;

    /** Scaling factor for linear standard deviation (distance-based uncertainty). */
    @Getter
    @Setter
    private double linearStdDevFactor = DEFAULT_LINEAR_STDDEV_FACTOR;

    /** Scaling factor for angular standard deviation (orientation-based uncertainty). */
    @Getter
    @Setter
    private double angularStdDevFactor = DEFAULT_ANGULAR_STDDEV_FACTOR;

    /**
     * Processes a {@link VisionObservation} containing multi-tag pose data from the coprocessor and
     * produces an estimated robot pose.
     *
     * <p>
     * If the observation does not include any tag detections or a valid multi-tag camera pose, this
     * method returns {@link Optional#empty()}.
     *
     * @param observation The current {@link VisionObservation} containing detected tags and an
     *        optional multi-tag camera pose.
     * @param heading The robot’s current field-relative heading. (Unused in this implementation.)
     * @return An {@link Optional} containing a {@link PoseRecord} with the estimated robot pose, or
     *         empty if no valid observation data was available.
     */
    @Override
    public Optional<PoseRecord> processVisionObservation(VisionObservation observation,
        Rotation2d heading) {

        var tags = observation.tagObservations();
        int tagCount = tags.size();
        Transform3d cameraToRobot = observation.camera().robotToCamera().inverse();
        var optionalMultiTagCameraPose = observation.multiTagCameraPose();

        // Ignore invalid observations
        if (observation.tagObservations().isEmpty() || optionalMultiTagCameraPose.isEmpty()) {
            return Optional.empty();
        }

        // Compute distance-based uncertainty scaling
        double avgDistance = tags.stream()
            .mapToDouble(tag -> tag.distance().in(Meters))
            .average().orElse(0.0);

        double stdDevFactor = Math.pow(avgDistance, 2.0) / tagCount;
        double linearStdDev = linearStdDevFactor * stdDevFactor;
        double angularStdDev = angularStdDevFactor * stdDevFactor;

        return Optional.of(
            new PoseRecord(
                optionalMultiTagCameraPose.get().plus(cameraToRobot),
                linearStdDev,
                angularStdDev));
    }
}
