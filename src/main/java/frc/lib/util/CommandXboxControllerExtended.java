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

    public CommandXboxControllerExtended(int port) {
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
     * @param side Which motor to rumble
     * @param intensity Percentage for rumble intensity
     * @return Command to rumble the controller
     */
    public Command rumble(RumbleType side, double intensity) {
        return Commands.startEnd(() -> hid.setRumble(side, intensity), () -> hid.setRumble(side, 0.0));
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
