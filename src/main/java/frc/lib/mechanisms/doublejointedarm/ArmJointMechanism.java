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
package frc.lib.mechanisms.doublejointedarm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import frc.lib.io.absoluteencoder.AbsoluteEncoderIO;
import frc.lib.io.absoluteencoder.AbsoluteEncoderInputsAutoLogged;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.Mechanism;
import frc.lib.util.PID;

import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public abstract class ArmJointMechanism<T extends MotorIO, E extends AbsoluteEncoderIO>
        extends Mechanism<T> {
    public final record JointCharacteristics(
            Distance armLength, Angle minAngle, Angle maxAngle, Angle startingAngle) {}

    protected final AbsoluteEncoderInputsAutoLogged absoluteEncoderInputs =
            new AbsoluteEncoderInputsAutoLogged();
    protected final Optional<E> absoluteEncoder;
    private final String encoderName;
    public final JointCharacteristics characteristics;
    private final PID pid;

    public ArmJointMechanism(
            String name,
            JointCharacteristics characteristics,
            T io,
            Optional<E> absoluteEncoder,
            String encoderName,
            PID pid) {
        super(name, io);
        this.absoluteEncoder = absoluteEncoder;
        this.encoderName = encoderName;
        this.characteristics = characteristics;
        this.pid = pid;
    }

    public Angle getPosition() {
        return inputs.position;
    }

    public void updatekg(double kg) {
        io.setPID(PIDSlot.SLOT_0, this.pid.withG(kg));
    }

    @Override
    public void periodic() {
        // First, refresh motor inputs from hardware in the base class.
        super.periodic();

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
