/*
 * Copyright (C) 2025 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */

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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.util.Device;
import lombok.Getter;
import frc.lib.util.CANUpdateThread;

/**
 * Abstraction for a CTRE TalonFX motor implementing the {@link MotorIO} interface. Wraps motor
 * setup, control modes, telemetry polling, and error handling.
 */
public class MotorIOTalonFX implements MotorIO {

    /**
     * Configuration data for a TalonFX motor that follows another TalonFX motor.
     * 
     * <p>
     * Follower motors mirror the output of a main motor, useful for mechanisms that require
     * multiple motors working together (like a dual-motor elevator).
     * 
     * @param id The CAN device ID of the follower motor
     * @param opposesMain Whether this follower should spin opposite to the main motor
     */
    public record TalonFXFollower(Device.CAN id, boolean opposesMain) {
    }

    @Getter
    protected final String name;

    protected final TalonFX motor;
    protected final TalonFX[] followers;

    // Cached signals for performance and easier access
    protected final StatusSignal<Angle> position;
    protected final StatusSignal<AngularVelocity> velocity;
    protected final StatusSignal<Voltage> supplyVoltage;
    protected final StatusSignal<Current> supplyCurrent;
    protected final StatusSignal<Current> torqueCurrent;
    protected final StatusSignal<Temperature> temperature;
    protected final StatusSignal<Double> closedLoopError;
    protected final StatusSignal<Double> closedLoopReference;
    protected final StatusSignal<Double> closedLoopReferenceSlope;

    // Preconfigured control objects reused for efficiency
    protected final CoastOut coastControl = new CoastOut();
    protected final StaticBrake brakeControl = new StaticBrake();
    protected final VoltageOut voltageControl = new VoltageOut(0).withEnableFOC(true);
    protected final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0);
    protected final DutyCycleOut dutyCycleControl = new DutyCycleOut(0).withEnableFOC(true);
    protected final DynamicMotionMagicTorqueCurrentFOC positionControl =
        new DynamicMotionMagicTorqueCurrentFOC(0, 0, 0,
            0);
    protected final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0);

    private final CANUpdateThread updateThread = new CANUpdateThread();

    private final Alert[] followerOnWrongBusAlert;

    protected Angle goalPosition = Rotations.of(0.0);

    /**
     * Constructs and initializes a TalonFX motor.
     * 
     * <p>
     * This constructor applies the provided configuration to the main motor and all followers. It
     * sets up the follower relationship, initializes status signals for telemetry, and configures
     * signal update rates. All followers must be on the same CAN bus as the main motor.
     * 
     * @param name The name of the motor(s) for logging and identification
     * @param config Configuration to apply to the motor(s) including PID, limits, and gear ratios
     * @param main CAN ID of the main motor
     * @param followerData Configuration data for the follower motor(s), can be empty if no
     *        followers
     */
    public MotorIOTalonFX(String name, TalonFXConfiguration config, Device.CAN main,
        TalonFXFollower... followerData)
    {
        this.name = name;

        motor = new TalonFX(main.id(), main.bus());
        updateThread.CTRECheckErrorAndRetry(() -> motor.getConfigurator().apply(config));

        // Initialize lists
        followerOnWrongBusAlert = new Alert[followerData.length];
        followers = new TalonFX[followerData.length];

        for (int i = 0; i < followerData.length; i++) {
            Device.CAN id = followerData[i].id();

            if (!id.bus().equals(main.bus())) {
                followerOnWrongBusAlert[i] =
                    new Alert(name + " follower " + i + " is on a different CAN bus than main!",
                        AlertType.kError);
                followerOnWrongBusAlert[i].set(true);
            }

            followers[i] = new TalonFX(id.id(), id.bus());

            TalonFX follower = followers[i];
            updateThread.CTRECheckErrorAndRetry(() -> follower.getConfigurator().apply(config));
            follower.setControl(new Follower(main.id(), followerData[i].opposesMain()));
        }

        position = motor.getPosition();
        velocity = motor.getVelocity();
        supplyVoltage = motor.getSupplyVoltage();
        supplyCurrent = motor.getSupplyCurrent();
        torqueCurrent = motor.getTorqueCurrent();
        temperature = motor.getDeviceTemp();
        closedLoopError = motor.getClosedLoopError();
        closedLoopReference = motor.getClosedLoopReference();
        closedLoopReferenceSlope = motor.getClosedLoopReferenceSlope();

        updateThread.CTRECheckErrorAndRetry(() -> BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            position,
            velocity,
            supplyCurrent,
            supplyCurrent,
            torqueCurrent,
            temperature));

        updateThread.CTRECheckErrorAndRetry(() -> BaseStatusSignal.setUpdateFrequencyForAll(
            200,
            closedLoopError,
            closedLoopReference,
            closedLoopReferenceSlope));

        motor.optimizeBusUtilization(0, 1.0);
    }

    /**
     * Checks if the motor is currently running a position control mode.
     * 
     * <p>
     * Position control modes command the motor to move to and hold a specific position. This
     * includes Motion Magic modes that use motion profiling.
     * 
     * @return true if the motor is using a position control mode
     */
    protected boolean isRunningPositionControl()
    {
        var control = motor.getAppliedControl();
        return (control instanceof PositionTorqueCurrentFOC)
            || (control instanceof PositionVoltage)
            || (control instanceof MotionMagicTorqueCurrentFOC)
            || (control instanceof DynamicMotionMagicTorqueCurrentFOC)
            || (control instanceof MotionMagicVoltage);
    }

    /**
     * Checks if the motor is currently running a velocity control mode.
     * 
     * <p>
     * Velocity control modes command the motor to spin at a specific speed.
     * 
     * @return true if the motor is using a velocity control mode
     */
    protected boolean isRunningVelocityControl()
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
     * <p>
     * Motion Magic is CTRE's advanced motion profiling control mode that automatically generates
     * smooth motion with controlled velocity, acceleration, and jerk (rate of acceleration).
     * 
     * @return true if the motor is using a Motion Magic mode
     */
    protected boolean isRunningMotionMagic()
    {
        var control = motor.getAppliedControl();
        return (control instanceof MotionMagicTorqueCurrentFOC)
            || (control instanceof DynamicMotionMagicTorqueCurrentFOC)
            || (control instanceof MotionMagicVelocityTorqueCurrentFOC)
            || (control instanceof MotionMagicVoltage)
            || (control instanceof MotionMagicVelocityVoltage);
    }

    /**
     * Determines the current control type being used by the motor.
     * 
     * <p>
     * This identifies which control mode the motor is currently in (coast, brake, voltage, current,
     * position, velocity, etc.).
     * 
     * @return The current ControlType
     */
    protected ControlType getCurrentControlType()
    {
        var control = motor.getAppliedControl();

        if (control instanceof StaticBrake) {
            return ControlType.BRAKE;
        } else if (control instanceof VoltageOut) {
            return ControlType.VOLTAGE;
        } else if (control instanceof TorqueCurrentFOC) {
            return ControlType.CURRENT;
        } else if (control instanceof DutyCycleOut) {
            return ControlType.DUTYCYCLE;
        } else if (isRunningPositionControl()) {
            return ControlType.POSITION;
        } else if (isRunningVelocityControl()) {
            return ControlType.VELOCITY;
        }

        return ControlType.COAST;
    }

    /**
     * Updates the passed-in MotorInputs structure with the latest sensor readings.
     * 
     * <p>
     * This method refreshes all status signals from the motor and populates the inputs object with
     * current telemetry data. The data varies based on the current control mode - for example,
     * position error and trajectory data are only populated during position control.
     * 
     * @param inputs Motor input structure to populate with current sensor data
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
        Double closedLoopErrorValue = closedLoopError.getValue();
        Double closedLoopTargetValue = closedLoopReference.getValue();

        boolean isRunningPositionControl = isRunningPositionControl();
        boolean isRunningMotionMagic = isRunningMotionMagic();
        boolean isRunningVelocityControl = isRunningVelocityControl();

        inputs.positionError = isRunningPositionControl
            ? Rotations.of(closedLoopErrorValue)
            : null;

        inputs.activeTrajectoryPosition =
            isRunningPositionControl && isRunningMotionMagic
                ? Rotations.of(closedLoopTargetValue)
                : null;

        inputs.goalPosition = isRunningPositionControl
            ? goalPosition
            : null;

        if (isRunningVelocityControl) {
            inputs.velocityError = RotationsPerSecond.of(closedLoopErrorValue);
            inputs.activeTrajectoryVelocity = RotationsPerSecond.of(closedLoopTargetValue);
        } else if (isRunningPositionControl && isRunningMotionMagic) {
            var targetVelocity = closedLoopReferenceSlope.getValue();
            inputs.velocityError = RotationsPerSecond.of(
                targetVelocity - inputs.velocity.in(RotationsPerSecond));
            inputs.activeTrajectoryVelocity = RotationsPerSecond.of(targetVelocity);
        } else {
            inputs.velocityError = null;
            inputs.activeTrajectoryVelocity = null;
        }

        inputs.controlType = getCurrentControlType();
    }

    /**
     * Sets the motor to coast mode (no active braking).
     * 
     * <p>
     * In coast mode, the motor spins freely when not powered. The mechanism will slow down
     * gradually due to friction but won't actively try to stop. Use this when you want the
     * mechanism to move freely.
     */
    @Override
    public void runCoast()
    {
        motor.setControl(coastControl);
    }

    /**
     * Sets the motor to brake mode (active braking).
     * 
     * <p>
     * In brake mode, the motor actively resists rotation when not powered. This causes the
     * mechanism to stop quickly and hold position. Use this when you need the mechanism to stay in
     * place.
     */
    @Override
    public void runBrake()
    {
        motor.setControl(brakeControl);
    }

    /**
     * Runs the motor using direct voltage control.
     * 
     * <p>
     * Voltage control directly applies a voltage to the motor.
     * 
     * @param voltage Desired voltage output (positive = forward, negative = reverse)
     */
    @Override
    public void runVoltage(Voltage voltage)
    {
        motor.setControl(voltageControl.withOutput(voltage));
    }

    /**
     * Runs the motor with a specified torque-producing current output.
     * 
     * <p>
     * Current control directly controls the current flowing through the motor, which is
     * proportional to torque output. This provides more consistent torque regardless of motor
     * speed, compared to voltage control.
     * 
     * @param current Desired torque-producing current (not total supply current)
     */
    @Override
    public void runCurrent(Current current)
    {
        motor.setControl(currentControl.withOutput(current).withMaxAbsDutyCycle(1.0));
    }

    /**
     * Runs the motor with a specified current output limited by duty cycle.
     * 
     * <p>
     * This version allows you to limit the maximum speed of the motor while using current control.
     * The duty cycle caps the percentage of available voltage.
     * 
     * @param current Desired torque-producing current
     * @param dutyCycle Maximum duty cycle (0.0 to 1.0) limiting top speed
     */
    @Override
    public void runCurrent(Current current, double dutyCycle)
    {
        double dutyCyclePercent = MathUtil.clamp(dutyCycle, 0.0, 1.0);
        motor.setControl(currentControl.withOutput(current).withMaxAbsDutyCycle(dutyCyclePercent));
    }

    /**
     * Runs the motor using duty cycle control (percentage of available voltage) with FOC.
     * 
     * <p>
     * Duty cycle control outputs a percentage of the current battery voltage. Unlike direct voltage
     * control, this automatically adjusts for battery voltage changes.
     * 
     * @param dutyCycle Fractional output between 0.0 and 1.0 (will be clamped to this range)
     */
    @Override
    public void runDutyCycle(double dutyCycle)
    {
        double dutyCyclePercent = MathUtil.clamp(dutyCycle, 0.0, 1.0);
        motor.setControl(dutyCycleControl.withOutput(dutyCyclePercent));
    }

    /**
     * Runs the motor to a specific position using Motion Magic with dynamic profiling.
     * 
     * <p>
     * Motion Magic generates a smooth motion profile with controlled cruise velocity, acceleration,
     * and jerk. Dynamic Motion Magic allows these parameters to be changed on-the-fly for each new
     * position command. This provides smooth, predictable motion that's easy on mechanisms and
     * looks professional.
     * 
     * @param position Target position to move to
     * @param cruiseVelocity Maximum velocity during the move
     * @param acceleration Maximum acceleration during speed-up and slow-down
     * @param maxJerk Maximum jerk (rate of acceleration change) for smoothness
     * @param slot PID slot index to use for position control gains
     */
    @Override
    public void runPosition(Angle position, AngularVelocity cruiseVelocity,
        AngularAcceleration acceleration,
        Velocity<AngularAccelerationUnit> maxJerk, PIDSlot slot)
    {
        this.goalPosition = position;
        motor.setControl(positionControl.withPosition(position).withVelocity(cruiseVelocity)
            .withAcceleration(acceleration).withJerk(maxJerk).withSlot(slot.getNum()));
    }

    /**
     * Runs the motor at a target velocity using FOC current control.
     * 
     * <p>
     * Velocity control maintains a constant speed using a PID controller. The motor will
     * automatically adjust output to maintain the target velocity as load changes.
     * 
     * @param velocity Desired angular velocity (positive = forward, negative = reverse)
     * @param acceleration Maximum acceleration when changing speeds
     * @param slot PID slot index to use for velocity control gains
     */
    @Override
    public void runVelocity(AngularVelocity velocity, AngularAcceleration acceleration,
        PIDSlot slot)
    {
        motor.setControl(
            velocityControl.withVelocity(velocity).withAcceleration(acceleration)
                .withSlot(slot.getNum()));
    }

    /**
     * Manually sets the encoder position to a specific value.
     * 
     * <p>
     * This is useful for zeroing the encoder when the mechanism is at a known position, or for
     * resetting position after hitting a hard stop. Use carefully as incorrect position can cause
     * the mechanism to move unexpectedly.
     * 
     * @param position The new position value to set the encoder to
     */
    @Override
    public void setEncoderPosition(Angle position)
    {
        motor.setPosition(position);
    }
}
