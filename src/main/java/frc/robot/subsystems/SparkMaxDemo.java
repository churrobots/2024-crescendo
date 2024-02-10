// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//new stuff
// import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

//old stuff
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.SubsystemInspector;
import frc.robot.helpers.Tunables.TunableDouble;
import frc.robot.helpers.Tunables.TunableInteger;

public class SparkMaxDemo extends SubsystemBase {

  private static final int deviceID = 1;
  private CANSparkMax m_motor;
  private SparkPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

  private static final class Constants {

    // private static final double calibrationVelocitySensorUnitsPerSecond = -3000;

    // private static final int offsetMaxCounts = 1000;

    // private static final int restingCounts = 3000;
    // private static final int aimSpeakerManCounts = 12400;
    // private static final int aimAMPCounts = 6500;
    // private static final int receiveIntakeFromGroundCounts = 18000;

    public static final TunableDouble kP = new TunableDouble("kP", 0.04);
    public static final TunableDouble kF = new TunableDouble("kF", 0.0); // 0.05
    public static final TunableDouble kI = new TunableDouble("kI", 0.0); // 0.000001
    public static final TunableDouble kD = new TunableDouble("kD", 0.0);

    public static final TunableDouble kMaxGravityFeedForward = new TunableDouble("kMaxGravityFeedForward", -0.07);
    public static final TunableInteger kArmSpeed = new TunableInteger("kArmSpeed", 6000);
    public static final TunableInteger kArmAcceleration = new TunableInteger("kArmAcceleration", 75000);
    public static final TunableInteger kArmSmoothing = new TunableInteger("kArmSmoothing", 1);

  }

  // private final SubsystemInspector m_inspector = new
  // SubsystemInspector(getSubsystem());
  // private Level level = Level.RESTING;

  // private final WPI_TalonFX armMotor = new WPI_TalonFX(Constants.armCanID);
  // private boolean m_isCalibrated = false;

  public SparkMaxDemo() {

    // initialize motor
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration
     * parameters
     * in the SPARK MAX to their factory default state. If no argument is passed,
     * these
     * parameters will not persist between power cycles
     */
    m_motor.restoreFactoryDefaults();

    // initialze PID controller and encoder objects
    m_pidController = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();
    // TODO: we probably want the separate absolute encoder
    // m_encoder = m_motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    // PID coefficients
    kP = 5e-5;
    kI = 1e-6;
    kD = 0;
    kIz = 0;
    kFF = 0.000156;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    // Smart Motion Coefficients
    maxVel = 2000; // rpm
    maxAcc = 1500;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    /**
     * Smart Motion coefficients are set on a SparkPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Max Velocity", maxVel);
    SmartDashboard.putNumber("Min Velocity", minVel);
    SmartDashboard.putNumber("Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Set Velocity", 0);

    // button to toggle between velocity and smart motion modes
    SmartDashboard.putBoolean("Mode", true);

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
    double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

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
    if ((maxV != maxVel)) {
      m_pidController.setSmartMotionMaxVelocity(maxV, 0);
      maxVel = maxV;
    }
    if ((minV != minVel)) {
      m_pidController.setSmartMotionMinOutputVelocity(minV, 0);
      minVel = minV;
    }
    if ((maxA != maxAcc)) {
      m_pidController.setSmartMotionMaxAccel(maxA, 0);
      maxAcc = maxA;
    }
    if ((allE != allowedErr)) {
      m_pidController.setSmartMotionAllowedClosedLoopError(allE, 0);
      allowedErr = allE;
    }

    double setPoint, processVariable;
    boolean mode = SmartDashboard.getBoolean("Mode", false);
    if (mode) {
      setPoint = SmartDashboard.getNumber("Set Velocity", 0);
      m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
      processVariable = m_encoder.getVelocity();
    } else {
      setPoint = SmartDashboard.getNumber("Set Position", 0);
      /**
       * As with other PID modes, Smart Motion is set by calling the
       * setReference method on an existing pid object and setting
       * the control type to kSmartMotion
       */
      m_pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
      processVariable = m_encoder.getPosition();
    }

    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("Process Variable", processVariable);
    SmartDashboard.putNumber("Output", m_motor.getAppliedOutput());
  }

  private void updateArmTuning() {
    // FalconUtils.configureMotionMagic(
    // armMotor,
    // Constants.kArmSpeed.get(),
    // Constants.kArmAcceleration.get(),
    // Constants.kArmSmoothing.get(),
    // Constants.kP.get(),
    // Constants.kF.get(),
    // Constants.kI.get(),
    // Constants.kD.get());
  }

}
