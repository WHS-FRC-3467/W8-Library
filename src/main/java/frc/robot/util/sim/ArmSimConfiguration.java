// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.sim;

/** Add your docs here. */
public class ArmSimConfiguration {

	public double kArmMass = 0.0; // Kilograms
	public double kArmLength = 0.0; // Meters
    public double kDefaultArmSetpointDegrees = 0.0; // Degrees
	public double kMinAngleDegrees = 0.0; // Degrees
	public double kMaxAngleDegrees = 0.0; // Degrees
	public double kArmReduction = 0.0; // RotorToSensorRatio * SensorToMechanismRatio
	public double kSensorReduction = 0.0; // SensorToMechanismRatio

}
