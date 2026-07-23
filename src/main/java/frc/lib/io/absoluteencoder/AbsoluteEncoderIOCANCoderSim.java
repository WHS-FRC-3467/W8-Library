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

package frc.lib.io.absoluteencoder;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.sim.CANcoderSimState;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;

import frc.lib.util.Device.CAN;

/**
 * Simulated hardware implementation of AbsoluteEncoderIO using CTRE CANcoder.
 *
 * <p>Interfaces with a CTRE CANcoder absolute magnetic encoder over the CAN bus. Provides
 * high-resolution absolute position sensing for mechanisms like swerve modules, arms, and turrets.
 * Uses Phoenix 6 API with signal-based updates.
 */
public class AbsoluteEncoderIOCANCoderSim extends AbsoluteEncoderIOCANCoder
        implements AbsoluteEncoderIOSim {

    private final CANcoderSimState simState;

    /**
     * Constructs a CANcoder interface with the specified configuration.
     *
     * @param id CAN device identifier (ID and bus name)
     * @param configuration CANcoder configuration including magnet offset and sensor direction
     * @param encoderToMechanismRatio Returns the ratio of measured encoder rotations to mechanism rotations. 1.0 means the encoder is mounted directly onto the output shaft of the mechanism. 
     */
    public AbsoluteEncoderIOCANCoderSim(CAN id, CANcoderConfiguration configuration, double sensorToMechanismRatio) {
        super(id, configuration, sensorToMechanismRatio);
        simState = CANCoder.getSimState();
    }

    @Override
    public void updateInputs(AbsoluteEncoderInputs inputs) {
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        super.updateInputs(inputs);
    }

    /**
     * Setter for the mechanism angle from which the encoder reading is simulated.
     *
     * @param angle The mechanism-space angle to simulate an encoder reading for
     */
    @Override
    public void setMechanismAngle(Angle angle) {
        simState.setRawPosition(angle.times(getEncoderToMechanismRatio()));
    }
}
