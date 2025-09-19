// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.mechanisms.linear;

import java.util.Optional;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Distance;
import frc.lib.io.motor.MotorIO.ControlType;
import frc.lib.io.motor.MotorInputsAutoLogged;
import frc.lib.mechanisms.Mechanism;
import frc.lib.util.MechanismUtil.DistanceAngleConverter;

/**
 * Abstract class for linear mechanisms, which are mechanisms that move in a straight line. This
 * class implements the Mechanism interface and provides characteristics specific to linear
 * mechanisms.
 */
public abstract class LinearMechanism implements Mechanism {

    public record LinearMechCharacteristics(
        Translation3d offset,
        Distance minDistance,
        Distance maxDistance,
        Distance startingDistance,
        DistanceAngleConverter converter) {
    }

    protected final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();
    protected final DistanceAngleConverter converter;

    private final LinearMechanismVisualizer visualizer;

    public LinearMechanism(String name, LinearMechCharacteristics characteristics)
    {
        visualizer = new LinearMechanismVisualizer(name, characteristics);
        converter = characteristics.converter();
    }

    private Optional<Distance> getTrajectoryDistance()
    {
        if (inputs.controlType != ControlType.POSITION || inputs.positionError == null) {
            return Optional.empty();
        }

        return Optional.of(converter.toDistance(inputs.activeTrajectoryPosition));
    }

    private Optional<Distance> getGoalDistance()
    {
        if (inputs.controlType != ControlType.POSITION || inputs.positionError == null) {
            return Optional.empty();
        }

        return Optional.of(converter.toDistance(inputs.position.plus(inputs.positionError)));
    }

    // Checks if mechanism is near a goal position within a specified tolerance
    public boolean nearGoal(Distance goalPosition, Distance tolerance)
    {
        return MathUtil.isNear(
            converter.toDistance(getPosition()).in(BaseUnits.DistanceUnit),
            goalPosition.in(BaseUnits.DistanceUnit),
            tolerance.in(BaseUnits.DistanceUnit));
    }

    @Override
    public void periodic()
    {
        visualizer.setMeasuredDistance(converter.toDistance(inputs.position));
        visualizer.setTrajectoryDistance(getTrajectoryDistance());
        visualizer.setGoalDistance(getGoalDistance());
    }
}
