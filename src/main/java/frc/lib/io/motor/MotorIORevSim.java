package frc.lib.io.motor;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import com.revrobotics.spark.SparkSim;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;


/**
 * Simulates a REV motor controller (SparkFlex or SparkMax) for FRC robot code.
 * <p>
 * Extends {@link MotorIORev} and implements {@link MotorIOSim} to enable simulation of leader and
 * follower motors, closed-loop control, and state updates for testing without hardware.
 * </p>
 *
 * <p>
 * Features:
 * <ul>
 * <li>Supports SparkFlex and SparkMax controllers</li>
 * <li>Configures followers with inversion</li>
 * <li>Simulates position and velocity control</li>
 * <li>Updates simulated inputs</li>
 * </ul>
 * </p>
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
        int id,
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
    public void runPosition(Angle position, AngularVelocity cruiseVelocity,
        AngularAcceleration acceleration,
        Velocity<AngularAccelerationUnit> maxJerk, PIDSlot slot)
    {
        simState.setPosition(position.in(Rotations));

    }


    @Override
    public void runVelocity(AngularVelocity velocity, AngularAcceleration acceleration,
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
        super.motor.close();
    }

}
