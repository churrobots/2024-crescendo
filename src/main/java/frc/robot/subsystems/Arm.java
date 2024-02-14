package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.CANMapping;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** A robot arm subsystem that moves with a motion profile. */
public class Arm extends TrapezoidProfileSubsystem {

  class Constants {

    // These are fake gains; in actuality these must be determined individually for
    // each robot
    // TODO: figure these out for our actual arm
    static final double kSVolts = 1;
    static final double kGVolts = 1;
    static final double kVVoltSecondPerRad = 0.5;
    static final double kAVoltSecondSquaredPerRad = 0.1;

    static final double kMaxVelocityRadPerSecond = 3;
    static final double kMaxAccelerationRadPerSecSquared = 10;

    // The offset of the arm from the horizontal in its neutral position,
    // measured from the horizontal
    static final double kArmOffsetRads = 0.5;

    // These are all the constants from the SparkMAX demo code.
    static final double kP = 0.1;
    static final double kI = 1e-4;
    static final double kD = 1;
    static final double kIz = 0;
    static final double kFF = 0;
    static final double kMaxOutput = 1;
    static final double kMinOutput = -1;
    static final int kCPR = 8192;

    // Default slot should be fine according to:
    // https://www.chiefdelphi.com/t/sparkmax-pid-controller/427438/4
    static final int defaultPidSlot = 0;
  }

  private final ArmFeedforward m_feedforward = new ArmFeedforward(
      Constants.kSVolts, Constants.kGVolts,
      Constants.kVVoltSecondPerRad, Constants.kAVoltSecondSquaredPerRad);

  private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;

  final CANSparkMax m_motor = new CANSparkMax(CANMapping.armMotor, MotorType.kBrushless);
  final SparkPIDController m_pidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  /**
   * An alternate encoder object is constructed using the GetAlternateEncoder()
   * method on an existing CANSparkMax object. If using a REV Through Bore
   * Encoder, the type should be set to quadrature and the counts per
   * revolution set to 8192
   */
  private RelativeEncoder m_alternateEncoder;

  /** Create a new ArmSubsystem. */
  public Arm() {
    super(
        new TrapezoidProfile.Constraints(
            Constants.kMaxVelocityRadPerSecond, Constants.kMaxAccelerationRadPerSecSquared),
        Constants.kArmOffsetRads);
    // TODO: replace this with equivalent code for SparkMAX
    // m_motor.setPID(ArmConstants.kP, 0, 0);

    // initialize SPARK MAX with CAN ID
    m_motor.restoreFactoryDefaults();

    m_alternateEncoder = m_motor.getAlternateEncoder(kAltEncType, Constants.kCPR);

    /**
     * In order to use PID functionality for a controller, a SparkPIDController
     * object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = m_motor.getPIDController();

    /**
     * By default, the PID controller will use the Hall sensor from a NEO for its
     * feedback device. Instead, we can set the feedback device to the alternate
     * encoder object
     */
    m_pidController.setFeedbackDevice(m_alternateEncoder);

    /**
     * From here on out, code looks exactly like running PID control with the
     * built-in NEO encoder, but feedback will come from the alternate encoder
     */

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    // TODO: figure out if this is appropriate feedforward for sparkmax?
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);

    /**
     * PIDController objects are commanded to a set point using the
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four
     * parameters:
     * com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     * com.revrobotics.CANSparkMax.ControlType.kPosition
     * com.revrobotics.CANSparkMax.ControlType.kVelocity
     * com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
    m_pidController.setReference(setpoint.position, CANSparkMax.ControlType.kPosition, Constants.defaultPidSlot,
        feedforward / 12.0);
  }

  public Command setArmGoalCommand(double kArmOffsetRads) {
    return Commands.runOnce(() -> setGoal(kArmOffsetRads), this);
  }

  @Override
  public void periodic() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((p != kP)) {
      m_pidController.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      m_pidController.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      m_pidController.setD(d);
      kD = d;
    }
    if ((iz != kIz)) {
      m_pidController.setIZone(iz);
      kIz = iz;
    }
    if ((ff != kFF)) {
      m_pidController.setFF(ff);
      kFF = ff;
    }
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      m_pidController.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;
    }

    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", m_alternateEncoder.getPosition());
  }
}