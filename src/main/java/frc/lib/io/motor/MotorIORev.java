package frc.lib.io.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.EnumMap;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.lib.util.Device;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class MotorIORev implements MotorIO {

    public SparkFlex motor;
    public SparkClosedLoopController controller;
    private RelativeEncoder encoder;

    private final String name;

    /**
     * REV WORKAROUNDS
     * 
     * The motorIO interface was made for CTRE hardware. Some code does not map 1:1 to REV. The
     * below are workarounds to make the mapping easier.
     */

    /**
     * Maps MotorIO.PIDSlot to REV ClosedLoopSlot.
     */
    private static final EnumMap<MotorIO.PIDSlot, ClosedLoopSlot> SLOT_MAP =
        new EnumMap<>(MotorIO.PIDSlot.class);
    static {
        SLOT_MAP.put(MotorIO.PIDSlot.SLOT_0, ClosedLoopSlot.kSlot0);
        SLOT_MAP.put(MotorIO.PIDSlot.SLOT_1, ClosedLoopSlot.kSlot1);
        SLOT_MAP.put(MotorIO.PIDSlot.SLOT_2, ClosedLoopSlot.kSlot2);
    }

    /**
     * Gets the corresponding REV ClosedLoopSlot for a given MotorIO.PIDSlot.
     * 
     * @param slot The MotorIO.PIDSlot to convert.
     */
    protected ClosedLoopSlot getClosedLoopSlot(MotorIO.PIDSlot slot)
    {
        return SLOT_MAP.getOrDefault(slot, ClosedLoopSlot.kSlot0);
    }

    /**
     * Maps MotorIO.ControlType to REV ControlType.
     */
    private static final EnumMap<ControlType, com.revrobotics.spark.SparkBase.ControlType> CONTROL_TYPE_MAP =
        new EnumMap<>(ControlType.class);
    static {
        CONTROL_TYPE_MAP.put(ControlType.VOLTAGE,
            com.revrobotics.spark.SparkBase.ControlType.kVoltage);
        CONTROL_TYPE_MAP.put(ControlType.CURRENT,
            com.revrobotics.spark.SparkBase.ControlType.kCurrent);
        CONTROL_TYPE_MAP.put(ControlType.DUTYCYCLE,
            com.revrobotics.spark.SparkBase.ControlType.kDutyCycle);
        CONTROL_TYPE_MAP.put(ControlType.POSITION,
            com.revrobotics.spark.SparkBase.ControlType.kPosition);
        CONTROL_TYPE_MAP.put(ControlType.VELOCITY,
            com.revrobotics.spark.SparkBase.ControlType.kVelocity);
    }

    /**
     * Gets the corresponding REV ControlType for a given MotorIO.ControlType.
     * 
     * @param type The MotorIO.ControlType to convert.
     */
    protected com.revrobotics.spark.SparkBase.ControlType getRevControlType(ControlType type)
    {
        return CONTROL_TYPE_MAP.getOrDefault(type,
            com.revrobotics.spark.SparkBase.ControlType.kDutyCycle);
    }



    /**
     * Constructor for a standalone leader motor. This motor is configured as the primary motor in a
     * system and can be manually controlled by the user. It also configures various settings for
     * motion profiling and closed-loop control.
     *
     * @param id The unique identifier for the motor (CAN ID).
     * @param isFlex Boolean indicating whether the motor is a SparkFlex (true) or a SparkMax
     *        (false).
     * @param gearRatio The gear ratio applied to the motor's output.
     * @param wheelRadius The radius of the wheel attached to the motor, used for motion
     *        calculations.
     * @param isBrake Boolean indicating whether the motor should use brake mode (true) or coast
     *        mode (false).
     * @param inverted Boolean indicating whether the motor should be inverted relative to the
     *        control signal.
     * @param motorType The type of motor (e.g., brushless, brushed) to configure.
     * @param motionProfile The motion profile settings for the motor, including PID and velocity
     *        parameters.
     *
     * @see MotorIO
     */
    public MotorIORev(
        String name,
        Device.CAN id,
        boolean isFlex,
        SparkBaseConfig config)
    {
        this.name = name;

        if (isFlex) {
            motor = new SparkFlex(id.id(), MotorType.kBrushless);
        } else {
            // motor = new SparkMax(id, MotorType.kBrushless);
        }

        controller = motor.getClosedLoopController();
        encoder = motor.getEncoder();

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public String getName()
    {
        return name;
    }

    @Override
    public void updateInputs(MotorInputs inputs)
    {
        inputs.connected = true;
        inputs.position = Rotation.of(encoder.getPosition());
        inputs.velocity = RotationsPerSecond.of(encoder.getVelocity() / 60.0);
        inputs.appliedVoltage = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
        inputs.supplyCurrent = Amps.of(motor.getOutputCurrent());
        inputs.temperature = Celsius.of(motor.getMotorTemperature());
        inputs.positionError = null;
        inputs.positionError = Angle.ofBaseUnits(0, Rotation);
        inputs.velocityError = AngularVelocity.ofBaseUnits(0, RotationsPerSecond);
        inputs.goalPosition = Angle.ofBaseUnits(0, Rotation);
        inputs.controlType = null;
    }

    @Override
    public void runBrake()
    {
        SparkBaseConfig newConfig = newEmptyConfig();
        newConfig.idleMode(IdleMode.kBrake);
        motor.configure(newConfig, ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters);
    }

    @Override
    public void runCoast()
    {
        SparkBaseConfig newConfig = newEmptyConfig();
        newConfig.idleMode(IdleMode.kCoast);
        motor.configure(newConfig, ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters);
    }

    @Override
    public void runVoltage(Voltage volts)
    {
        controller.setReference(volts.in(Volts), getRevControlType(ControlType.VOLTAGE));
    }

    @Override
    public void runCurrent(Current current)
    {
        controller.setReference(current.in(Amps), getRevControlType(ControlType.CURRENT));
    }

    @Override
    public void runDutyCycle(double percent)
    {
        motor.set(percent);
    }

    @Override
    public void runPosition(Angle position, AngularVelocity cruiseVelocity,
        AngularAcceleration acceleration,
        Velocity<AngularAccelerationUnit> maxJerk, PIDSlot slot)
    {
        controller.setReference(position.in(Rotation), getRevControlType(ControlType.POSITION),
            getClosedLoopSlot(slot));
    }


    @Override
    public void runVelocity(AngularVelocity velocity, AngularAcceleration acceleration,
        PIDSlot slot)
    {
        controller.setReference(velocity.in(RotationsPerSecond) * 60,
            getRevControlType(ControlType.VELOCITY),
            getClosedLoopSlot(slot));
    }

    @Override
    public void setEncoderPosition(Angle position)
    {
        encoder.setPosition(position.in(Rotation));

    }

    private SparkBaseConfig newEmptyConfig()
    {
        return (motor instanceof SparkFlex) ? new SparkFlexConfig() : new SparkMaxConfig();
    }

    protected void close()
    {
        this.motor.close();
    }

    protected SparkFlex getMotor()
    {
        return this.motor;
    }

}
