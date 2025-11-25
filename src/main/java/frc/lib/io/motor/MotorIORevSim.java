package frc.lib.io.motor;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import com.revrobotics.spark.SparkSim;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.util.Device;


/**
 * Simulated implementation of {@link MotorIORev} for REV Robotics motors using WPILib simulation.
 * Implements {@link MotorIOSim} to provide simulation-specific behavior.
 *
 * <p>
 * Constructor arguments:
 * <ul>
 * <li><b>name</b> - Name of the motor</li>
 * <li><b>id</b> - CAN ID of the motor</li>
 * <li><b>isFlex</b> - True if using SparkFlex, false for SparkMax</li>
 * <li><b>gearBox</b> - DCMotor gearbox model</li>
 * <li><b>config</b> - Motor configuration</li>
 * <li><b>followerData</b> - Varargs of follower motor data (ID and inversion)</li>
 * </ul>
 *
 * <p>
 * This class wraps a simulated SparkFlex or SparkMax motor, allowing position and velocity control
 * in a simulation environment. It provides methods to run the motor in position or velocity mode,
 * update simulation inputs, and manage simulation state.
 *
 * @see MotorIORev
 * @see MotorIOSim
 */
public class MotorIORevSim extends MotorIORev implements MotorIOSim {

    public record RevFollowerFollower(int id, boolean opposesLeader) {
    }

    public SparkFlex motor;
    public SparkClosedLoopController controller;
    private SparkSim simState;

    /**
     * Constructs a MotorIORevSim instance.
     * 
     * @param name Name of the motor
     * @param id CAN ID of the motor
     * @param isFlex True if using SparkFlex, false for SparkMax
     * @param gearBox DCMotor gearbox model
     * @param config Motor configuration
     * @param followerData Varargs of follower motor data (ID and inversion)
     * 
     * @see MotorIO
     */
    public MotorIORevSim(
        String name,
        Device.CAN id,
        boolean isFlex,
        DCMotor gearBox,
        SparkBaseConfig config,
        RevFollowerFollower... followerData)
    {
        super(name, id, isFlex, config);

        motor = this.getMotor();

        if (isFlex) {
            simState = new SparkFlexSim(motor, gearBox);
        } else {
            // motor = new SparkMax(id, MotorType.kBrushless);
        }


        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        simState.enable();
    }

    @Override
    public void runPosition(Angle position, PIDSlot slot)
    {
        simState.setPosition(position.in(Rotations));

    }


    @Override
    public void runVelocity(AngularVelocity velocity,
        PIDSlot slot)
    {
        simState.setVelocity(velocity.in(RotationsPerSecond) * 60);

    }

    @Override
    public double getRotorToSensorRatio()
    {
        return 1;
    }

    public double getSensorToMechanismRatio()
    {
        return 1;
    }


    @Override
    public void updateInputs(MotorInputs inputs)
    {

        simState.setBusVoltage(12.0);

        simState.iterate(
            simState.getVelocity(),
            simState.getBusVoltage(),
            0.02);

        inputs.position = Rotation.of(motor.getEncoder().getPosition());
        inputs.velocity = RotationsPerSecond.of(motor.getEncoder().getVelocity() / 60);
        inputs.appliedVoltage = Volts.of(simState.getAppliedOutput() * simState.getBusVoltage());
        inputs.supplyCurrent = Amps.of(simState.getMotorCurrent());
        inputs.temperature = Celsius.of(0);

    }

    @Override
    public void close()
    {
        super.close();
    }

}
