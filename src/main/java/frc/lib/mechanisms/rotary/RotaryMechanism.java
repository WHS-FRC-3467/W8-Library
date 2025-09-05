// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.mechanisms.rotary;

import frc.lib.io.motor.MotorIO.ControlType;
import frc.lib.io.motor.MotorInputsAutoLogged;
import frc.lib.mechanisms.Mechanism;
import java.util.Optional;
import edu.wpi.first.math.geometry.Translation3d;
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

    private RotaryVisualizer visualizer;

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

    @Override
    public void periodic()
    {
        visualizer.setCurrentAngle(inputs.position);
        visualizer.setTrajectoryAngle(getTrajectoryAngle());
        visualizer.setGoalAngle(getGoalAngle());
    }
}
