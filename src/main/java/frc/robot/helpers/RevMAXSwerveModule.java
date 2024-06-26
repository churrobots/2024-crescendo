// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.helpers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;

/**
 * Build on top of the Rev MAX sample code
 * https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/subsystems/MAXSwerveModule.java
 * More RevLib examples:
 * https://github.com/REVrobotics/SPARK-MAX-Examples/tree/master/Java
 */
public class RevMAXSwerveModule {

  private static final class Constants {

    // Tuning values
    public static final double kWheelDiameterMeters = 0.0831;

    public static final double kDrivingP = .08;// .04
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;

    public static final double kTurningP = 2;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;

    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    // Note that the module itself has 45 teeth on the wheel's bevel gear, 22 teeth
    // on the first-stage spur gear, 15 teeth on the bevel pinion.
    public static final int kDrivingMotorPinionTeeth = 16;
    public static final int kWheelBevelTeeth = 45;
    public static final int kFirstStageSpurTeeth = 20;
    public static final int kBevelPinionTeeth = 15;

    public static final double kMotorFreeSpeedRpm = 5676; // Neo motors are 5676 max RPM
  }

  private final CANSparkMax m_drivingSparkMax;
  private final CANSparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkMaxPIDController m_drivingPIDController;
  private final SparkMaxPIDController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  // Invert the turning encoder, since the output shaft rotates in the opposite
  // direction of
  // the steering motor in the MAXSwerve Module.
  private final boolean kTurningEncoderInverted = true;

  // Calculations required for driving motor conversion factors and feed forward
  private final double kDrivingMotorFreeSpeedRps = Constants.kMotorFreeSpeedRpm / 60;

  private final double kWheelCircumferenceMeters = Constants.kWheelDiameterMeters * Math.PI;

  // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
  // teeth on the bevel pinion
  private final double kDrivingMotorReduction = ((double) Constants.kWheelBevelTeeth * Constants.kFirstStageSpurTeeth)
      / ((double) Constants.kDrivingMotorPinionTeeth * Constants.kBevelPinionTeeth);
  private final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
      / kDrivingMotorReduction;

  private final double kDrivingEncoderPositionFactor = (Constants.kWheelDiameterMeters * Math.PI)
      / kDrivingMotorReduction; // meters
  private final double kDrivingEncoderVelocityFactor = ((Constants.kWheelDiameterMeters * Math.PI)
      / kDrivingMotorReduction) / 60.0; // meters per second

  private final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
  private final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

  private final double kTurningEncoderPositionPIDMinInput = 0; // radians
  private final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

  private final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
  private final double kDrivingMinOutput = -1;
  private final double kDrivingMaxOutput = 1;

  private final double kTurningFF = 0;
  private final double kTurningMinOutput = -1;
  private final double kTurningMaxOutput = 1;

  private final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
  private final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

  private final int kDrivingMotorCurrentLimit = 40; // 40amps
  private final int kTurningMotorCurrentLimit = 20; // amps

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public RevMAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drivingSparkMax.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_drivingPIDController = m_drivingSparkMax.getPIDController();
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_drivingEncoder.setPositionConversionFactor(kDrivingEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    m_turningEncoder.setInverted(kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!
    m_drivingPIDController.setP(Constants.kDrivingP);
    m_drivingPIDController.setI(Constants.kDrivingI);
    m_drivingPIDController.setD(Constants.kDrivingD);
    m_drivingPIDController.setFF(kDrivingFF);
    m_drivingPIDController.setOutputRange(kDrivingMinOutput,
        kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(Constants.kTurningP);
    m_turningPIDController.setI(Constants.kTurningI);
    m_turningPIDController.setD(Constants.kTurningD);
    m_turningPIDController.setFF(kTurningFF);
    m_turningPIDController.setOutputRange(kTurningMinOutput,
        kTurningMaxOutput);

    m_drivingSparkMax.setIdleMode(kDrivingMotorIdleMode);
    m_turningSparkMax.setIdleMode(kTurningMotorIdleMode);
    m_drivingSparkMax.setSmartCurrentLimit(kDrivingMotorCurrentLimit);
    m_turningSparkMax.setSmartCurrentLimit(kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingSparkMax.burnFlash();
    m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    // TODO: should this be using the optimizedDesiredState? is this a bug?
    m_desiredState = desiredState;
    // TODO: uncomment this to add debug logging for tuning
    // _debug(optimizedDesiredState)
  }

  void _debug(SwerveModuleState optimizedDesiredState) {
    SmartDashboard.putNumber("Actual Velocity" +
        m_drivingSparkMax.getDeviceId(), m_drivingEncoder.getVelocity());
    SmartDashboard.putNumber("Desired Velocity" +
        m_drivingSparkMax.getDeviceId(),
        optimizedDesiredState.speedMetersPerSecond);
    SmartDashboard.putNumber("Actual Angle" +
        m_turningSparkMax.getDeviceId(), m_turningEncoder.getPosition());
    SmartDashboard.putNumber("Desired Angle" + m_turningSparkMax.getDeviceId(),
        optimizedDesiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset)).getRadians());

  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetDriveEncodersToZero() {
    m_drivingEncoder.setPosition(0);
  }

  public void assertModuleIsPointedForwardAndStoreCalibration() {
    // TODO: store calibration state based on CAN ID
  }
}