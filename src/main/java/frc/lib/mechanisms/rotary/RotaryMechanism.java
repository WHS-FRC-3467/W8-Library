// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.mechanisms.rotary;

import frc.lib.io.motor.MotorIO.ControlType;
import frc.lib.io.motor.MotorInputsAutoLogged;
import frc.lib.mechanisms.Mechanism;
import java.util.Optional;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public abstract class RotaryMechanism implements Mechanism {

    public static record RotaryMechCharacteristics(
        Translation3d offset,
        Distance armLength,
        Angle minAngle,
        Angle maxAngle,
        Angle startingAngle) {
    }

    protected final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();

    private final RotaryVisualizer visualizer;

    public RotaryMechanism(String name, RotaryMechCharacteristics characteristics)
    {
        visualizer = new RotaryVisualizer(name, characteristics);
    }

    private Optional<Angle> getTrajectoryAngle()
    {
        if (inputs.controlType != ControlType.POSITION || inputs.positionError == null) {
            return Optional.empty();
        }

        return Optional.of(inputs.activeTrajectoryPosition);
    }

    private Optional<Angle> getGoalAngle()
    {
        if (inputs.controlType != ControlType.POSITION || inputs.positionError == null) {
            return Optional.empty();
        }

        return Optional.of(inputs.position.plus(inputs.positionError));
    }

    // Checks if mechanism is near a goal position within a specified tolerance
    public boolean nearGoal(Angle goalAngle, Angle tolerance)
    {
        return MathUtil.isNear(
            getPosition().in(BaseUnits.AngleUnit),
            goalAngle.in(BaseUnits.AngleUnit),
            tolerance.in(BaseUnits.AngleUnit));
    }

    @Override
    public void periodic()
    {
        visualizer.setCurrentAngle(inputs.position);
        visualizer.setTrajectoryAngle(getTrajectoryAngle());
        visualizer.setGoalAngle(getGoalAngle());
    }
}
