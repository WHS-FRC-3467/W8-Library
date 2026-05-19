// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.mechanisms.differential;

import edu.wpi.first.units.measure.Angle;

import frc.lib.io.absoluteencoder.AbsoluteEncoderIO;
import frc.lib.io.absoluteencoder.AbsoluteEncoderInputsAutoLogged;
import frc.lib.io.motor.MotorIO;
import frc.lib.mechanisms.Mechanism;

import java.util.Optional;

/** Add your docs here. */
public abstract class DifferentialMechanism<T extends MotorIO, E extends AbsoluteEncoderIO>
        extends Mechanism<T> {

    // Controlled by the difference between the two motors
    public enum DifferenceAxis {
        PITCH,
        YAW,
        ROLL,
        X,
        Y,
        Z
    }

    // Controlled by the average of the two motors
    public enum AverageAxis {
        PITCH,
        YAW,
        ROLL,
        X,
        Y,
        Z
    }

    public static record DiffMechCharacteristics(
            DifferenceAxis differenceAxis,
            AverageAxis averageAxis,
            double gearing,
            Angle startingDifference,
            Angle startingAverage) {}

    protected final AbsoluteEncoderInputsAutoLogged absoluteEncoderInputs =
            new AbsoluteEncoderInputsAutoLogged();
    protected final Optional<E> absoluteEncoder;
    private final String encoderName;

    protected DifferentialMechanism(
            String name,
            DiffMechCharacteristics characteristics,
            T io,
            Optional<E> absoluteEncoder,
            String encoderName) {
        super(name, io);
        this.absoluteEncoder = absoluteEncoder;
        this.encoderName = encoderName;
        // TODO Auto-generated constructor stub
    }
}
