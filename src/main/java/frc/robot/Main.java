/*
 * Copyright (c) 2009-2025 FIRST and other WPILib contributors All rights reserved.
 */

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
    private Main() {}

    /**
     * Main initialization function. Do not perform any initialization here.
     *
     * <p>If you change your main robot class, change the parameter type.
     */
    public static void main(String... args) {

        // Source - https://stackoverflow.com/a/34839209
        // Posted by TWiStErRob, modified by community. See post 'Timeline' for change history
        // Retrieved 2026-06-19, License - CC BY-SA 3.0

      
        RobotBase.startRobot(Robot::new);
    }
}
