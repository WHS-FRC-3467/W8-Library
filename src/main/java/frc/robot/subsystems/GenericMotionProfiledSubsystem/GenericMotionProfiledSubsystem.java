package frc.robot.subsystems.GenericMotionProfiledSubsystem;

import frc.robot.util.Alert;
import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class GenericMotionProfiledSubsystem<G extends GenericMotionProfiledSubsystem.TargetState>
    extends SubsystemBase {

  // Tunable numbers
  private static LoggedTunableNumber kP, kI, kD,
                                     kG, kS, kV, kA,
                                     kCruiseVelocity, kAcceleration, kJerk;

  public enum ProfileType {
    POSITION,
    VELOCITY,
    MM_POSITION,
    MM_VELOCITY,
    OPEN_VOLTAGE,
    OPEN_CURRENT
  }

  public interface TargetState {

    public double getOutput();
    public double getFeedFwd();
  }

  public abstract G getState();

  private final String mName;
  private final ProfileType mProType;
  private final GenericMotionProfiledSubsystemConstants mConstants;
  private final GenericMotionProfiledSubsystemIO io;
  protected final GenericMotionProfiledIOInputsAutoLogged inputs = new GenericMotionProfiledIOInputsAutoLogged();
  private final Alert leaderMotorDisconnected;
  private final Alert followerMotorDisconnected;
  private final Alert CANcoderDisconnected;

  private final boolean debug = true;

  public GenericMotionProfiledSubsystem(ProfileType pType,
                                        GenericMotionProfiledSubsystemConstants constants,
                                        GenericMotionProfiledSubsystemIO io) {
    this.mProType = pType;
    this.mConstants = constants;
    this.io = io;
    this.mName = mConstants.kName;

    this.leaderMotorDisconnected = new Alert(mName + " Leader motor disconnected!", Alert.AlertType.WARNING);
    this.followerMotorDisconnected = new Alert(mName + " Follower motor disconnected!", Alert.AlertType.WARNING);
    this.CANcoderDisconnected = new Alert(mName + " CANcoder disconnected!", Alert.AlertType.WARNING);

    // Tunable numbers for PID and motion gain constants
    kP = new LoggedTunableNumber(mName + "/Gains/kP", mConstants.kMotorConfig.Slot0.kP);
    kI = new LoggedTunableNumber(mName + "/Gains/kI", mConstants.kMotorConfig.Slot0.kI);
    kD = new LoggedTunableNumber(mName + "/Gains/kD", mConstants.kMotorConfig.Slot0.kD);
    
    kG = new LoggedTunableNumber(mName + "/Gains/kG", mConstants.kMotorConfig.Slot0.kG);
    kS = new LoggedTunableNumber(mName + "/Gains/kS", mConstants.kMotorConfig.Slot0.kS);
    kV = new LoggedTunableNumber(mName + "/Gains/kV", mConstants.kMotorConfig.Slot0.kV);
    kA = new LoggedTunableNumber(mName + "/Gains/kA", mConstants.kMotorConfig.Slot0.kA);

    kCruiseVelocity = new LoggedTunableNumber(mName + "/CruiseVelocity",
        mConstants.kMotorConfig.MotionMagic.MotionMagicCruiseVelocity);
    kAcceleration = new LoggedTunableNumber(mName + "/Acceleration",
        mConstants.kMotorConfig.MotionMagic.MotionMagicAcceleration);
    kJerk = new LoggedTunableNumber(mName + "/Jerk",
        mConstants.kMotorConfig.MotionMagic.MotionMagicJerk);

    io.configurePID(0, kP.get(), kI.get(), kD.get(), 0.0, true);
    io.configureGSVA(kG.get(), kS.get(), kV.get(), kA.get(), true);
    io.configureMotion(kCruiseVelocity.get(), kAcceleration.get(), kJerk.get(), true);
  }

  public void periodic() {

    io.updateInputs(inputs);
    Logger.processInputs(mName, inputs);

    // Check for disconnections
    leaderMotorDisconnected.set(!inputs.leaderMotorConnected);
    followerMotorDisconnected.set(!inputs.followerMotorConnected);
    CANcoderDisconnected.set(!inputs.CANcoderConnected);

    // If changed, update controller constants from Tuneable Numbers
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.configurePID(0, kP.get(), kI.get(), kD.get(), 0.0, false),
        kP,
        kI,
        kD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.configureGSVA(kG.get(), kS.get(), kV.get(), kA.get(), false),
        kG,
        kS,
        kV,
        kA);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.configureMotion(kCruiseVelocity.get(), kAcceleration.get(), kJerk.get(), false),
        kCruiseVelocity,
        kAcceleration,
        kJerk);

    // Run system based on Profile Type
    switch (mProType) {
      default:
      case POSITION:
        /* Run Closed Loop to position in rotations */
        io.runToPosition(getState().getOutput(), getState().getFeedFwd());
        break;
      case VELOCITY:
        /* Run Closed Loop to velocity in rotations/second */
        io.runToVelocity(getState().getOutput(), getState().getFeedFwd());
        break;
      case MM_POSITION:
        /* Run Motion Magic to the specified position setpoint (in rotations) */
        io.runMotionMagicPosition(getState().getOutput(), getState().getFeedFwd());
        break;
      case MM_VELOCITY:
        /* Run Motion Magic to the specified velocity setpoint (in rotations/second) */
        io.runMotionMagicVelocity(getState().getOutput(), getState().getFeedFwd());
        break;
      case OPEN_VOLTAGE:
        /* Run Open Loop to velocity in rotations/second */
        io.runVoltage(getState().getOutput());
        break;
      case OPEN_CURRENT:
        /* Run Open Loop to current in amps */
        io.runCurrent(getState().getOutput());
        break;
    }

    Logger.recordOutput(mName + "/Goal", getState().toString());
    
    displayInfo(debug);
  }

  private void displayInfo(boolean debug) {
    if (debug) {
      SmartDashboard.putNumber(mName + " Output", inputs.appliedVoltage[0]);
      SmartDashboard.putNumber(mName + " Current Draw", inputs.supplyCurrentAmps[0]);
    }
  }

}
