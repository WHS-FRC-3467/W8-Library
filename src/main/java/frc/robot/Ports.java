// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public enum Ports {

    CANDLE(30, "rio"); // Placeholders

    public final int id;
    public final String bus;

    private Ports(int id, String bus) {
        this.id = id;
        this.bus = bus;
    }
}
