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

package frc.robot.subsystems.DetectionML;

import frc.lib.io.DetectionML.DetectionMLIO;
import frc.lib.io.DetectionML.DetectionMLIOAutoLogged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DetectionML extends SubsystemBase {
    private final DetectionMLIO[] io;
    private final DetectionMLIOAutoLogged[] inputs;

    public DetectionML(DetectionMLIO io)
    {
        this.inputs = io;

        // Initialize inputs
        this.inputs = new DetectionMLIOAutoLogged[io.length]; // here
        for (

            int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }
    }
}
