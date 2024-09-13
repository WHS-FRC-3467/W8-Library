package frc.robot.subsystems.SimpleSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface SimpleSubsystemIO {
    @AutoLog
    public static class SimpleSubsystemIOInputs {
        // This is where we will establish variables
        public double test = 0.0;
    }

    /* Updates the set of loggable inputs */
    public default void updateInputs(SimpleSubsystemIOInputs inputs) {}

    /* Run Open Loop at the specified voltage. */
    public default void setVoltage(double volts) {}

    /* Stop in open loop */
    public default void stop() {}

    /* Set velocity PID constants */
    public default void configurePID(double kP, double kI, double kD) {}
}
