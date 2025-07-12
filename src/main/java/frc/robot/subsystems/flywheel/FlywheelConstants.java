// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.lib.io.motor.MotorIOTalonFX;
import frc.lib.io.motor.MotorIOTalonFXSim;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.flywheel.FlywheelMechanismReal;
import frc.lib.mechanisms.flywheel.FlywheelMechanismSim;
import frc.lib.mechanisms.flywheel.FlywheelMechanismSim.PhysicsException;
import frc.robot.Ports;

/** Add your docs here. */
public class FlywheelConstants {

    public static final String NAME = "Flywheel";
    public static final TalonFXConfiguration CONFIG = new TalonFXConfiguration();

    public static final DCMotor CHARACTERISTICS = DCMotor.getKrakenX60(1);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(1);

    public static FlywheelMechanismReal getReal()
    {
        return new FlywheelMechanismReal(new MotorIOTalonFX(NAME, CONFIG, Ports.flywheel));
    }

    public static FlywheelMechanismSim getSim()
    {
        try {
            return new FlywheelMechanismSim(new MotorIOTalonFXSim(NAME, CONFIG, Ports.flywheel),
                CHARACTERISTICS, MOI);
        } catch (PhysicsException e) {
            throw new IllegalStateException(e);
        }
    }

    public static FlywheelMechanism getReplay()
    {
        return new FlywheelMechanism() {};
    }
}
