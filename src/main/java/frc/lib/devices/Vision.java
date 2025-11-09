/*
 * Copyright (C) 2025 Windham Windup
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

package frc.lib.devices;

import java.util.List;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.vision.VisionIO;
import frc.lib.io.vision.VisionIOInputsAutoLogged;
import frc.lib.io.vision.VisionIO.VisionObservation;

public class Vision extends SubsystemBase {
    private final String name;
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    private final Consumer<VisionObservation> visionConsumer;

    public Vision(String name, VisionIO io, Consumer<VisionObservation> visionConsumer) {
        this.name = name;
        this.io = io;
        this.visionConsumer = visionConsumer;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);

        if (!inputs.connected)
            return;

        List.of(inputs.poseObservations).forEach(visionConsumer);
    }
}
