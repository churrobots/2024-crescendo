package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.CANMapping;
import frc.robot.helpers.Tunables;
import frc.robot.helpers.Tunables.TunableDouble;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** A robot arm subsystem that moves with a motion profile. */
public class Arm extends TrapezoidProfileSubsystem {

  class Constants {

    static final double kMaxVelocityRadPerSecond = 1;
    static final double kMaxAccelerationRadPerSecSquared = 1;
    static final TrapezoidProfile.Constraints trapezoidProfile = new TrapezoidProfile.Constraints(
        Constants.kMaxVelocityRadPerSecond, Constants.kMaxAccelerationRadPerSecSquared);

    // The offset of the arm from the horizontal in its neutral position,
    // measured from the horizontal
    static final double kArmOffsetRads = 0;

    // These are all the constants from the SparkMAX demo code.
    static final TunableDouble kP = new TunableDouble("armKP", 6);
    static final TunableDouble kI = new TunableDouble("armKI", 0);
    static final TunableDouble kD = new TunableDouble("armKD", 0);
    static final double kIz = 0;
    static final double kFF = 0;
    static final double kMaxOutput = 0.7;
    static final double kMinOutput = -0.7;

    // These are all the constants from the sample WPIlib trapezoid subsystem code.
    static final TunableDouble kSVolts = new TunableDouble("feedFowardSVolts", 0);
    static final TunableDouble kGVolts = new TunableDouble("feedFowardGVolts", 0.335);
    static final TunableDouble kVVoltSecondPerRad = new TunableDouble("feedFowardVVoltSecondPerRad", 6.24);
    static final TunableDouble kAVoltSecondSquaredPerRad = new TunableDouble("feedFowardAVoltSecondSquaredPerRad",
        0.04);

    // These constants are from the sample WPIlib code and shouldn't need to change.
    static final int kCPR = 8192;

    // Default slot should be fine according to:
    // https://www.chiefdelphi.com/t/sparkmax-pid-controller/427438/4
    static final int defaultPidSlot = 0;

    static final TunableDouble groundPosition = new TunableDouble("groundPosition", 0); // tune
    static final TunableDouble ampPosition = new TunableDouble("ampPosition", 0.09); // tune
    static final TunableDouble speakerPosition = new TunableDouble("speakerPosition", 0.2); // tune
    static final TunableDouble defaultPosition = new TunableDouble("defaultPosition", 0); // tune
  }

  final CANSparkMax right_motor = new CANSparkMax(CANMapping.rightArmMotor, MotorType.kBrushless);
  final CANSparkMax left_motor = new CANSparkMax(CANMapping.leftArmMotor, MotorType.kBrushless);

  final SparkPIDController m_pidController;

  final ArmFeedforward m_feedforward = new ArmFeedforward(
      Constants.kSVolts.get(),
      Constants.kGVolts.get(),
      Constants.kVVoltSecondPerRad.get(),
      Constants.kAVoltSecondSquaredPerRad.get());

  /**
   * An alternate encoder object is constructed using the GetAlternateEncoder()
   * method on an existing CANSparkMax object. If using a REV Through Bore
   * Encoder, the type should be set to quadrature and the counts per
   * revolution set to 8192
   */
  final SparkAbsoluteEncoder m_absoluteEncoder;

  /** Create a new ArmSubsystem. */
  // We used some sample code from:
  // https://github.com/Delmar-Robotics-Engineers-At-MADE/2024-Robot/blob/main/src/main/java/frc/robot/subsystems/Arm.java
  public Arm() {
    super(Constants.trapezoidProfile, Constants.kArmOffsetRads);

    right_motor.restoreFactoryDefaults();
    right_motor.setSmartCurrentLimit(40);
    right_motor.setIdleMode(IdleMode.kBrake);
    right_motor.setInverted(false);
    m_absoluteEncoder = right_motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    m_pidController = right_motor.getPIDController();
    m_pidController.setFeedbackDevice(m_absoluteEncoder);
    // TODO: do we need to enable PID wrapping?
    // m_pidController.setPositionPIDWrappingEnabled(true);

    left_motor.restoreFactoryDefaults();
    left_motor.setSmartCurrentLimit(40);
    left_motor.setIdleMode(IdleMode.kBrake);
    left_motor.setInverted(true);
    left_motor.follow(right_motor, true);

    m_pidController.setP(Constants.kP.get());
    m_pidController.setI(Constants.kI.get());
    m_pidController.setD(Constants.kD.get());
    m_pidController.setIZone(Constants.kIz);
    // m_pidController.setFF(Constants.kFF);
    m_pidController.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);

    enable();
    right_motor.burnFlash();
    left_motor.burnFlash();
    SmartDashboard.putNumber("Arm:mostRecentSetpointPosition", 0);
    SmartDashboard.putNumber("Arm:mostRecentFeedForward", 0);
  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
    double feedforward = m_feedforward.calculate(
        setpoint.position * 2 * Math.PI,
        setpoint.velocity);
    // boolean lowEnoughToRest = m_absoluteEncoder.getPosition() < 0.02 &&
    // setpoint.position < 0.01;
    // if (lowEnoughToRest) {
    // right_motor.stopMotor();
    // left_motor.stopMotor();
    // } else {
    m_pidController.setReference(
        setpoint.position,
        CANSparkMax.ControlType.kPosition,
        Constants.defaultPidSlot,
        feedforward);
    // }
    // SmartDashboard.putBoolean("Arm:lowEnoughToRest", lowEnoughToRest);
    SmartDashboard.putNumber("Arm:mostRecentSetpointPosition", setpoint.position);
    SmartDashboard.putNumber("Arm:mostRecentFeedForward", feedforward);
  }

  public void move_speaker() {
    SmartDashboard.putNumber("Arm:goalRotations", Constants.speakerPosition.get());
    setGoal(Constants.speakerPosition.get());
  }

  public void move_amp() {
    SmartDashboard.putNumber("Arm:goalRotations", Constants.ampPosition.get());
    setGoal(Constants.ampPosition.get());
    // right_motor.set(0.2);
  }

  public void move_ground() {
    SmartDashboard.putNumber("Arm:goalRotations", Constants.groundPosition.get());
    setGoal(Constants.groundPosition.get());
  }

  public void move_Default() {
    // move_ground();
    right_motor.stopMotor();
    left_motor.stopMotor();
  }

  public boolean inRangeOf(double targetPosition) {
    double position = m_absoluteEncoder.getPosition();
    double tolerance = 0.05;
    if ((position > (targetPosition - tolerance)) && (position < (targetPosition + tolerance))) {
      return true;
    }
    return false;
  }

  public boolean inRangeOfSpeakerPosition() {
    return inRangeOf(Constants.speakerPosition.get());
  }

  public boolean inRangeOfAmpPosition() {
    return inRangeOf(Constants.ampPosition.get());
  }

  public boolean inRangeOfGroundPosition() {
    return inRangeOf(Constants.groundPosition.get());
  }

  public boolean inRangeOfDefaultPosition() {
    return inRangeOf(Constants.defaultPosition.get());
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("Arm:encoder", m_absoluteEncoder.getPosition());
    SmartDashboard.putNumber("Arm:appliedOutputRight", right_motor.getAppliedOutput());
    SmartDashboard.putNumber("Arm:appliedOutputLeft", left_motor.getAppliedOutput());
  }

}