package frc.robot.util.sim.mechanisms;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Class to keep all the mechanism-specific objects together and out of the main example
 */
public class GenericRollerSimMechanism {
    
    String mSimName;
    Mechanism2d mMech;
    private List<MechanismLigament2d> mRollers = new ArrayList<MechanismLigament2d>();

    public GenericRollerSimMechanism(String simName, int numRollers) {

        double HEIGHT = .50; // Controls the height of the mech2d SmartDashboard
        double WIDTH = .50; // Controls width of the mech2d SmartDashboard

        mSimName = simName;
        mMech = new Mechanism2d(WIDTH * numRollers, HEIGHT);

        for (int i = 0; i < numRollers; i++) {

            //Position
            MechanismLigament2d roller = mMech.
                getRoot("pivotPoint_" + i, WIDTH/2.0 + (i * WIDTH), HEIGHT / 2.0).
                append(new MechanismLigament2d("roller " + i, .2, 0, 0, new Color8Bit(Color.kAliceBlue)));

            MechanismLigament2d side1 = roller.append(new MechanismLigament2d("side1 " + i, 0.15307, 112.5, 6, new Color8Bit(Color.kAliceBlue)));
            MechanismLigament2d side2 = side1.append(new MechanismLigament2d("side2 " + i, 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
            MechanismLigament2d side3 = side2.append(new MechanismLigament2d("side3 " + i, 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
            MechanismLigament2d side4 = side3.append(new MechanismLigament2d("side4 " + i, 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
            MechanismLigament2d side5 = side4.append(new MechanismLigament2d("side5 " + i, 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
            MechanismLigament2d side6 = side5.append(new MechanismLigament2d("side6 " + i, 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
            MechanismLigament2d side7 = side6.append(new MechanismLigament2d("side7 " + i, 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
            side7.append(new MechanismLigament2d("side8 " + i, 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));

            mRollers.add(roller);
        }            
    }

    /**
     * Runs the mech2d widget in GUI.
     */
    public void update(int index, double position) {
        
        mRollers.get(index).setAngle(Units.rotationsToDegrees(position));
        SmartDashboard.putData(mSimName, mMech); // Creates mech2d in SmartDashboard
    }
}
