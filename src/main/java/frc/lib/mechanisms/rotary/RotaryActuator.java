// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.mechanisms.rotary;

import frc.lib.io.absoluteencoder.AbsoluteEncoderIO;
import frc.lib.io.absoluteencoder.AbsoluteEncoderInputsAutoLogged;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorIO.ControlType;
import frc.lib.mechanisms.Actuator;

import org.littletonrobotics.junction.Logger;
import org.wpilib.math.util.MathUtil;
import org.wpilib.units.BaseUnits;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

import java.util.Optional;

/**
 * Abstract base class for rotary mechanisms that pivot around an axis. Examples include arms,
 * hoods, and pivots. Supports position control with optional absolute encoder feedback and 3D
 * visualization.
 *
 * @param <T> the type of MotorIO implementation used by this mechanism
 * @param <E> the type of AbsoluteEncoderIO implementation (if used)
 */
public abstract class RotaryActuator<T extends MotorIO, E extends AbsoluteEncoderIO>
        extends Actuator<T> {

    public enum RotaryAxis {
        PITCH,
        YAW,
        ROLL
    }

    public static record RotaryMechCharacteristics(
            Distance armLength,
            Angle minAngle,
            Angle maxAngle,
            Angle startingAngle,
            RotaryAxis axis) {}

    protected final AbsoluteEncoderInputsAutoLogged absoluteEncoderInputs =
            new AbsoluteEncoderInputsAutoLogged();
    protected final Optional<E> absoluteEncoder;
    private final String encoderName;

    private final RotaryActuatorVisualizer visualizer;

    public RotaryActuator(
            String name,
            RotaryMechCharacteristics characteristics,
            T io,
            Optional<E> absoluteEncoder,
            String encoderName) {
        super(name, io);
        this.absoluteEncoder = absoluteEncoder;
        this.encoderName = encoderName;
        visualizer = new RotaryActuatorVisualizer(name, characteristics);
    }

    private Optional<Angle> getTrajectoryAngle() {
        if (inputs.controlType != ControlType.POSITION || inputs.positionError == null) {
            return Optional.empty();
        }

        return Optional.of(inputs.activeTrajectoryPosition);
    }

    private Optional<Angle> getGoalAngle() {
        if (inputs.controlType != ControlType.POSITION || inputs.positionError == null) {
            return Optional.empty();
        }

        return Optional.of(inputs.goalPosition);
    }

    // Checks if mechanism is near a goal position within a specified tolerance
    public boolean nearGoal(Angle goalAngle, Angle tolerance) {
        return MathUtil.isNear(
                getPosition().in(BaseUnits.AngleUnit),
                goalAngle.in(BaseUnits.AngleUnit),
                tolerance.in(BaseUnits.AngleUnit));
    }

    @Override
    public void periodic() {
        // First, refresh motor inputs from hardware in the base class.
        super.periodic();

        // Then, use the up-to-date inputs to update the visualizer.
        visualizer.setCurrentAngle(inputs.position);
        visualizer.setTrajectoryAngle(getTrajectoryAngle());
        visualizer.setGoalAngle(getGoalAngle());

        // Finally, update and log absolute encoder inputs if present.
        absoluteEncoder.ifPresent(
                encoder -> {
                    encoder.updateInputs(absoluteEncoderInputs);
                    Logger.processInputs(encoderName, absoluteEncoderInputs);
                });
    }

    @Override
    public void close() {
        super.close();
        absoluteEncoder.ifPresent(AbsoluteEncoderIO::close);
    }
}
