// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.sim.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class to draw a simulated Elevator mechanism controlled by motion profiling
 */
public class MotionProfiledElevatorMechanism {

    String mSimName;
    Mechanism2d mMech;
    MechanismLigament2d mElevator;

    public MotionProfiledElevatorMechanism(String simName) {
    
        double HEIGHT = 50; // Controls the height of the mech2d SmartDashboard
        double WIDTH = 20; // Controls width of the mech2d SmartDashboard

        mSimName = simName;

        // Create a Mechanism2d visualization of the elevator
        mMech = new Mechanism2d(WIDTH, HEIGHT);
        MechanismRoot2d mech2dRoot = mMech.getRoot("Elevator Root", 10, 0);
        mElevator =  mech2dRoot.append(
                new MechanismLigament2d(
                    "Elevator",
                    0.0,
                    90));

    }


    /**
     * Runs the mech2d widget in GUI.
     */
    public void update(double position) {

        mElevator.setLength(position);
        SmartDashboard.putData(mSimName, mMech); // Creates mech2d in SmartDashboard
    }
}
