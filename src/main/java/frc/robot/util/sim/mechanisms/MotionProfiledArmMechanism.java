package frc.robot.util.sim.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Class to draw a simulated Arm mechanism controlled by motion profiling
 */
public class MotionProfiledArmMechanism {

    String mSimName;
    Mechanism2d mMech;
    MechanismLigament2d mArm;

    public MotionProfiledArmMechanism(String simName) {
    
        double HEIGHT = 60; // Controls the height of the mech2d SmartDashboard
        double WIDTH = 60; // Controls width of the mech2d SmartDashboard

        mSimName = simName;

        // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
        mMech = new Mechanism2d(WIDTH, HEIGHT);
        MechanismRoot2d m_armPivot = mMech.getRoot("ArmPivot", 30, 30);
        m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
        mArm = m_armPivot.append(
                new MechanismLigament2d(
                        "Arm",
                        30,
                        0,
                        6,
                        new Color8Bit(Color.kYellow)));

    }


    /**
     * Runs the mech2d widget in GUI.
     */
    public void update(double degrees) {

        mArm.setAngle(degrees);
        SmartDashboard.putData(mSimName, mMech); // Creates mech2d in SmartDashboard
    }
}
