// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.io;

/** Add your docs here. */
public abstract class MotorIO {

    public class Inputs {}

    public enum Mode {
        IDLE,
		VOLTAGE,
        DUTY_CYCLE,
        POSITION,
        VELOCITY,
		PROFILE;
    }

    public static class Setpoint {}

}

