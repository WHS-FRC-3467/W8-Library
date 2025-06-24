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

package frc.lib.io.distancesensor;

import static edu.wpi.first.units.Units.Meters;
import frc.lib.util.LoggedTunableNumber;
import lombok.Getter;

/**
 * A simulated distance sensor implementation
 */
public class DistanceSensorIOSim implements DistanceSensorIO {
    @Getter
    private final String name;

    private final LoggedTunableNumber distance;

    /**
     * Constructs a new {@link DistanceSensorIOSim} with specified parameters and configuration.
     *
     * @param name A human-readable name for the sensor instance.
     */
    public DistanceSensorIOSim(String name)
    {
        this.name = name;

        distance = new LoggedTunableNumber(name + "Meters: ", 0.0);
    }

    @Override
    public void updateInputs(DistanceSensorInputs inputs)
    {
        inputs.ambientSignal = 0.0;
        inputs.connected = true;
        inputs.distance = Meters.of(distance.get());
    }
}
