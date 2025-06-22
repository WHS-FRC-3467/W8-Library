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

package frc.lib.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CommandXboxControllerExtended extends CommandXboxController {
    private GenericHID hid;
    private double deadband = 0.0;

    public CommandXboxControllerExtended(int port)
    {
        super(port);
        hid = this.getHID();
    }

    /**
     * Apply a deadband to all sticks
     * 
     * @param deadband The percent deadband to apply
     * @return this
     */
    public CommandXboxControllerExtended withDeadband(double deadband)
    {
        this.deadband = deadband;
        return this;
    }

    /**
     * Rumble controller until command ends
     * 
     * @param side Which motor to rumble
     * @param intensity Percentage for rumble intensity
     * @return Command to rumble the controller
     */
    public Command rumble(RumbleType side, double intensity)
    {
        return Commands.startEnd(() -> hid.setRumble(side, intensity),
            () -> hid.setRumble(side, 0.0));
    }

    @Override
    public double getLeftX()
    {
        return MathUtil.applyDeadband(super.getLeftX(), deadband);
    }

    @Override
    public double getLeftY()
    {
        return MathUtil.applyDeadband(super.getLeftY(), deadband);
    }

    @Override
    public double getRightX()
    {
        return MathUtil.applyDeadband(super.getRightX(), deadband);
    }

    @Override
    public double getRightY()
    {
        return MathUtil.applyDeadband(super.getRightY(), deadband);
    }
}
