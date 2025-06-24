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

package frc.lib.io.lights;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANdle;
import frc.lib.util.CANUpdateThread;
import frc.lib.util.Device;
import lombok.Getter;

public class LightsIOCandle implements LightsIO {
    @Getter
    private final String name;

    private final CANdle candle;

    private final CANUpdateThread updateThread = new CANUpdateThread();

    public LightsIOCandle(String name, Device.CAN id, CANdleConfiguration config)
    {
        this.name = name;

        candle = new CANdle(id.id(), id.bus());

        updateThread.CTRECheckErrorAndRetry(() -> candle.getConfigurator().apply(config));
    }

    @Override
    public void setAnimation(ControlRequest request)
    {
        candle.setControl(request);
    }

}
