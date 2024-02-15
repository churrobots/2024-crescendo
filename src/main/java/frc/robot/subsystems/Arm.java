package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

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

    // These are fake gains; in actuality these must be determined individually for
    // each robot
    // TODO: figure these out for our actual arm
    // static final double kSVolts = 1;
    // static final double kGVolts = 1;
    // static final double kVVoltSecondPerRad = 0.5;
    // static final double kAVoltSecondSquaredPerRad = 0.1;
    // static final ArmFeedforward m_feedforward = new ArmFeedforward(
    // Constants.kSVolts, Constants.kGVolts,
    // Constants.kVVoltSecondPerRad, Constants.kAVoltSecondSquaredPerRad);

    static final double kMaxVelocityRadPerSecond = 3;
    static final double kMaxAccelerationRadPerSecSquared = 10;
    static final TrapezoidProfile.Constraints trapezoidProfile = new TrapezoidProfile.Constraints(
        Constants.kMaxVelocityRadPerSecond, Constants.kMaxAccelerationRadPerSecSquared);
    // The offset of the arm from the horizontal in its neutral position,
    // measured from the horizontal
    static final double kArmOffsetRads = 0;

    // These are all the constants from the SparkMAX demo code.
    static final TunableDouble kP = new TunableDouble("armKP", 1.5); // tune
    static final TunableDouble kI = new TunableDouble("armKI", 0); // tune
    static final TunableDouble kD = new TunableDouble("armKD", 0); // tune
    static final double kIz = 0;
    static final double kFF = 0;
    static final double kMaxOutput = 1;
    static final double kMinOutput = -1;
    static final int kCPR = 8192;
    static final SparkAbsoluteEncoder.Type kAltEncType = SparkAbsoluteEncoder.Type.kDutyCycle;

    // Default slot should be fine according to:
    // https://www.chiefdelphi.com/t/sparkmax-pid-controller/427438/4
    static final int defaultPidSlot = 0;
  }

  final CANSparkMax m_motor = new CANSparkMax(CANMapping.armMotor, MotorType.kBrushless);
  final SparkPIDController m_pidController;

  /**
   * An alternate encoder object is constructed using the GetAlternateEncoder()
   * method on an existing CANSparkMax object. If using a REV Through Bore
   * Encoder, the type should be set to quadrature and the counts per
   * revolution set to 8192
   */
  final SparkAbsoluteEncoder m_absoluteEncoder;

  /** Create a new ArmSubsystem. */
  public Arm() {
    super(Constants.trapezoidProfile, Constants.kArmOffsetRads);

    m_motor.restoreFactoryDefaults();
    m_absoluteEncoder = m_motor.getAbsoluteEncoder(Constants.kAltEncType);
    m_pidController = m_motor.getPIDController();
    m_pidController.setFeedbackDevice(m_absoluteEncoder);

    /**
     * From here on out, code looks exactly like running PID control with the
     * built-in NEO encoder, but feedback will come from the alternate encoder
     */

    // TODO: replace this with equivalent code for SparkMAX
    // m_motor.setPID(ArmConstants.kP, 0, 0);
    // set PID coefficients
    m_pidController.setP(Constants.kP.get());
    m_pidController.setI(Constants.kI.get());
    m_pidController.setD(Constants.kD.get());
    m_pidController.setIZone(Constants.kIz);
    m_pidController.setFF(Constants.kFF);
    m_pidController.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);
    enable();
  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    // TODO: figure out if this is appropriate feedforward for sparkmax?
    // double feedforward = Constants.m_feedforward.calculate(setpoint.position,
    // setpoint.velocity);
    // m_pidController.setReference(setpoint.position,
    // CANSparkMax.ControlType.kPosition, Constants.defaultPidSlot,
    // feedforward / 12.0);
    SmartDashboard.putNumber("SetPoint:Rotations", setpoint.position);
    m_pidController.setReference(setpoint.position, ControlType.kPosition);
  }

  public void move_Close() {
    SmartDashboard.putNumber("Goal:Rotations", 0.5);
    setGoal(.5);
  }

  public void move_Default() {
    setGoal(0);
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("EncoderPosition", m_absoluteEncoder.getPosition());
    m_pidController.setP(Constants.kP.get());
    m_pidController.setI(Constants.kI.get());
    m_pidController.setD(Constants.kD.get());
  }
}