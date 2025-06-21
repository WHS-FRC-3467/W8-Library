package frc.lib.io.motor;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.util.CANDevice;
import frc.lib.util.CTREUpdateThread;

/**
 * Abstraction for a CTRE TalonFX motor implementing the {@link Motor} interface. Wraps motor setup,
 * control modes, telemetry polling, and error handling.
 */
public class MotorTalonFX implements Motor {
    private final CANDevice id;
    private final TalonFX motor;

    // Cached signals for performance and easier access
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> supplyVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> temperature;
    private final StatusSignal<Double> closedLoopError;
    private final StatusSignal<Double> closedLoopReference;
    private final StatusSignal<Double> closedLoopReferenceSlope;

    // Preconfigured control objects reused for efficiency
    private final CoastOut coastControl = new CoastOut();
    private final StaticBrake brakeControl = new StaticBrake();
    private final Follower followControl = new Follower(0, false);
    private final VoltageOut voltageControl = new VoltageOut(0).withEnableFOC(true);
    private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0);
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0).withEnableFOC(true);
    private final DynamicMotionMagicTorqueCurrentFOC positionControl =
        new DynamicMotionMagicTorqueCurrentFOC(0, 0, 0,
            0);
    private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0);

    private final CTREUpdateThread updateThread = new CTREUpdateThread();

    /**
     * Constructs and initializes a TalonFX motor.
     *
     * @param id CAN bus ID and bus name.
     * @param config Configuration to apply to the motor.
     */
    public MotorTalonFX(CANDevice id, TalonFXConfiguration config)
    {
        this.id = id;
        motor = new TalonFX(id.id(), id.bus());

        updateThread.checkErrorAndRetry(() -> motor.getConfigurator().apply(config));

        position = motor.getPosition();
        velocity = motor.getVelocity();
        supplyVoltage = motor.getSupplyVoltage();
        supplyCurrent = motor.getSupplyCurrent();
        torqueCurrent = motor.getTorqueCurrent();
        temperature = motor.getDeviceTemp();
        closedLoopError = motor.getClosedLoopError();
        closedLoopReference = motor.getClosedLoopReference();
        closedLoopReferenceSlope = motor.getClosedLoopReferenceSlope();

        updateThread.checkErrorAndRetry(() -> BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            position,
            velocity,
            supplyCurrent,
            supplyCurrent,
            torqueCurrent,
            temperature));

        updateThread.checkErrorAndRetry(() -> BaseStatusSignal.setUpdateFrequencyForAll(
            200,
            closedLoopError,
            closedLoopReference,
            closedLoopReferenceSlope));

        motor.optimizeBusUtilization(0, 1.0);
    }

    /**
     * Checks if the motor is currently running a position control mode.
     *
     * @return True if the motor is using a position control mode.
     */
    private boolean isRunningPositionControl()
    {
        var control = motor.getAppliedControl();
        return (control instanceof PositionTorqueCurrentFOC)
            || (control instanceof PositionVoltage)
            || (control instanceof MotionMagicTorqueCurrentFOC)
            || (control instanceof MotionMagicVoltage);
    }

    /**
     * Checks if the motor is currently running a velocity control mode.
     *
     * @return True if the motor is using a velocity control mode.
     */
    private boolean isRunningVelocityControl()
    {
        var control = motor.getAppliedControl();
        return (control instanceof VelocityTorqueCurrentFOC)
            || (control instanceof VelocityVoltage)
            || (control instanceof MotionMagicVelocityTorqueCurrentFOC)
            || (control instanceof MotionMagicVelocityVoltage);
    }

    /**
     * Checks if the motor is running any Motion Magic mode.
     *
     * @return True if the motor is using a Motion Magic mode.
     */
    private boolean isRunningMotionMagic()
    {
        var control = motor.getAppliedControl();
        return (control instanceof MotionMagicTorqueCurrentFOC)
            || (control instanceof MotionMagicVelocityTorqueCurrentFOC)
            || (control instanceof MotionMagicVoltage)
            || (control instanceof MotionMagicVelocityVoltage);
    }

    /**
     * Updates the passed-in MotorInputs structure with the latest sensor readings.
     *
     * @param inputs Motor input structure to populate.
     */
    @Override
    public void updateInputs(MotorInputs inputs)
    {
        inputs.connected = BaseStatusSignal.refreshAll(
            position,
            velocity,
            supplyVoltage,
            supplyCurrent,
            torqueCurrent,
            temperature,
            closedLoopError,
            closedLoopReference,
            closedLoopReferenceSlope)
            .isOK();

        inputs.position = position.getValue();
        inputs.velocity = velocity.getValue();
        inputs.appliedVoltage = supplyVoltage.getValue();
        inputs.supplyCurrent = supplyCurrent.getValue();
        inputs.torqueCurrent = torqueCurrent.getValue();
        inputs.temperature = temperature.getValue();

        // Interpret control-loop status signals conditionally based on current mode
        var closedLoopErrorValue = closedLoopError.getValue();
        var closedLoopTargetValue = closedLoopReference.getValue();

        inputs.positionError = isRunningPositionControl()
            ? Rotations.of(closedLoopErrorValue)
            : null;

        inputs.activeTrajectoryPosition = isRunningPositionControl() && isRunningMotionMagic()
            ? Rotations.of(closedLoopTargetValue)
            : null;

        if (isRunningVelocityControl()) {
            inputs.velocityError = RotationsPerSecond.of(closedLoopErrorValue);
            inputs.activeTrajectoryVelocity = RotationsPerSecond.of(closedLoopTargetValue);
        } else if (isRunningPositionControl() && isRunningMotionMagic()) {
            var targetVelocity = closedLoopReferenceSlope.getValue();
            inputs.velocityError = RotationsPerSecond.of(
                targetVelocity - inputs.velocity.in(RotationsPerSecond));
            inputs.activeTrajectoryVelocity = RotationsPerSecond.of(targetVelocity);
        } else {
            inputs.velocityError = null;
            inputs.activeTrajectoryVelocity = null;
        }
    }

    /**
     * Returns the motor's CAN device ID.
     *
     * @return The CANDevice representing this motor's ID and bus name.
     */
    @Override
    public CANDevice getID()
    {
        return id;
    }

    /**
     * Sets the motor to coast mode.
     */
    @Override
    public void runCoast()
    {
        motor.setControl(coastControl);
    }

    /**
     * Sets the motor to brake mode.
     */
    @Override
    public void runBrake()
    {
        motor.setControl(brakeControl);
    }

    /**
     * Follows the specified motor using CAN follower mode.
     *
     * @param followMotor The motor to follow.
     * @param oppose Whether or not to oppose the main motor.
     */
    @Override
    public void follow(Motor followMotor, boolean oppose)
    {
        motor.setControl(
            followControl.withMasterID(followMotor.getID().id()).withOpposeMasterDirection(oppose));
    }

    /**
     * Runs the motor using direct voltage control.
     *
     * @param voltage Desired voltage output.
     */
    @Override
    public void runVoltage(Voltage voltage)
    {
        motor.setControl(voltageControl.withOutput(voltage));
    }

    /**
     * Runs the motor with a specified current output.
     *
     * @param current Desired torque-producing current.
     */
    @Override
    public void runCurrent(Current current)
    {
        motor.setControl(currentControl.withOutput(current));
    }

    /**
     * Runs the motor using duty cycle (percentage of available voltage).
     *
     * @param dutyCycle Fractional output between 0 and 1.
     */
    @Override
    public void runDutyCycle(double dutyCycle)
    {
        double dutyCyclePercent = MathUtil.clamp(dutyCycle, 0.0, 1.0);
        motor.setControl(dutyCycleControl.withOutput(dutyCyclePercent));
    }

    /**
     * Runs the motor to a specific position.
     *
     * @param position Target position.
     * @param cruiseVelocity Cruise velocity.
     * @param acceleration Max acceleration.
     * @param maxJerk Max jerk (rate of acceleration).
     * @param slot PID slot index.
     */
    @Override
    public void runPosition(Angle position, AngularVelocity cruiseVelocity,
        AngularAcceleration acceleration,
        Velocity<AngularAccelerationUnit> maxJerk, PIDSlot slot)
    {
        motor.setControl(positionControl.withPosition(position).withVelocity(cruiseVelocity)
            .withAcceleration(acceleration).withJerk(maxJerk).withSlot(slot.ordinal()));
    }

    /**
     * Runs the motor at a target velocity.
     *
     * @param velocity Desired velocity.
     * @param acceleration Max acceleration.
     * @param slot PID slot index.
     */
    @Override
    public void runVelocity(AngularVelocity velocity, AngularAcceleration acceleration,
        PIDSlot slot)
    {
        motor.setControl(
            velocityControl.withVelocity(velocity).withAcceleration(acceleration)
                .withSlot(slot.ordinal()));
    }
}
