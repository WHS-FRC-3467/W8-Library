package frc.robot.subsystems.GenericMotionProfiledSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface GenericMotionProfiledSubsystemIO {
  @AutoLog
  abstract class GenericMotionProfiledIOInputs {
  
    // Flags to indicate which components are active and connected
    public boolean leaderMotorConnected = false;
    public boolean followerMotorConnected = false;
    public boolean CANcoderConnected = false;

    public double positionRot = 0.0;
    public double velocityRps = 0.0;
    public double[] appliedVoltage = new double[] {0.0, 0.0};
    public double[] supplyCurrentAmps = new double[] {0.0, 0.0};
    public double[] torqueCurrentAmps = new double[] {0.0, 0.0};
    public double[] tempCelsius = new double[] {0.0, 0.0};

    public double errorRotations = 0.0;
    public double activeTrajectoryPosition = 0.0;
    public double activeTrajectoryVelocity = 0.0;

    public double absoluteEncoderPositionRot = 0.0;
    public double relativeEncoderPositionRot = 0.0;

  }

  default void updateInputs(GenericMotionProfiledIOInputs inputs) {
  }

  /** Run Open Loop at the specified voltage */
  public default void runVoltage(double volts) {
  }

  /** Run Open Loop at the specified current */
  public default void runCurrent(double amps) {
  }

  /** Run Closed Loop to position in rotations */
  public default void runToPosition(double position, double feedforward) {
  }

  /** Run Closed Loop to velocity in rotations/second */
  public default void runToVelocity(double velocity, double feedforward) {
  }

  /** Run Motion Magic to the specified setpoint */
  public default void runMotionMagicPosition(double setpoint, double feedFwd) {
  }

  /** Run Motion Magic to the specified velocity */
  public default void runMotionMagicVelocity(double velocity, double feedFwd) {
  }

  /* Stop in Open Loop */
  public default void stop() {
  }

  /* Configure PID constants */
  public default void configurePID(int slot, double kP, double kI, double kD, double kV, boolean check) {
  }

  /* Configure Closed Loop constants */
  public default void configureGSVA(double kG, double kS, double kV, double kA, boolean check) {
  }

  /* Configure Motion constants */
  public default void configureMotion(double kCruise, double kAccel, double kJerk, boolean check) {
  }

  /* Get the closed loop operational setpoint (in rotations) */
  public default double getSetpoint() {
    return 0;
  }

  /* Get current mechanism position (in rotations) */
  public default double getPosition() {
    return 0;
  }

  /* Get current mechanism velocity (in rotations per second) */
  public default double getVelocity() {
    return 0;
  }

  /* Has the closed loop completed (within tolerance)? */
  public default boolean hasFinishedTrajectory(double tolerance) {
    return false;
  }

}
