package frc.robot.subsystems.SimpleSubsystem;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class SimpleSubsystemIOTalonFX implements SimpleSubsystemIO {
    /* This section will define the motor characterization as well as the settings we want to apply */
    private final TalonFX leader = new TalonFX(Constants.ExampleSubsystemConstants.ID_Motor); //
    //private final TalonFX follower = new TalonFX(1); // Not using a follower

    private final StatusSignal<Double> leaderPosition = leader.getPosition(); //
    private final StatusSignal<Double> leaderVelocity = leader.getVelocity(); // 
    private final StatusSignal<Double> leaderAppliedVolts = leader.getMotorVoltage(); //
    private final StatusSignal<Double> leaderCurrent = leader.getSupplyCurrent(); //
    //private final StatusSignal<Double> followerCurrent = follower.getSupplyCurrent(); //

    public SimpleSubsystemIOTalonFX() {
        var config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast; //Coast || Brake
        leader.getConfigurator().apply(config);
        //follower.getConfigurator().apply(config);
        //follower.setControl(new Follower(leader.getDeviceID(), false));

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent/*, followerCurrent*/);
        leader.optimizeBusUtilization();
        //follower.optimizeBusUtilization();
    }

    // Now we will override those methods form the IO Layer again
    @Override
    public void updateInputs(SimpleSubsystemIOInputs inputs) {
        // you always want to make sure you are updating the global variables that carry out through the IO layers
        // Here we are making sure that we also update the Hardware as well
        BaseStatusSignal.refreshAll(
        leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent/*, followerCurrent */);

        inputs.test = 0.2;
    }

    @Override
    public void setVoltage(double volts) {
        leader.setControl(new VoltageOut(volts));
    }

    @Override
    public void stop() {
        leader.stopMotor();
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        var config = new Slot0Configs();
        config.kP = kP;
        config.kI = kI;
        config.kD = kD;
        leader.getConfigurator().apply(config);
    }
}
