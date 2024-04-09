// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CANMapping;
import frc.robot.helpers.RevMAXSwerveModule;
import frc.robot.helpers.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private static final class Constants {

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(18.5);
    public static final double kWheelBase = Units.inchesToMeters(23.5);

    // Autonomous constants
    public static final double autoTranslationP = 10.0; // TODO: other team had 3,0,0
    public static final double autoTranslationI = 0.0;
    public static final double autoTranslationD = 0.0;

    public static final double autoRotationP = 2.0; // TODO: other team had 7,0,0.01
    public static final double autoRotationI = 0.0;
    public static final double autoRotationD = 0.0;

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kRearLeftChassisAngularOffset = Math.PI;
    public static final double kRearRightChassisAngularOffset = Math.PI / 2;

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 6.04;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kDriveBaseRadiusInMeters = Math
        .sqrt(Math.pow(kTrackWidth / 2, 2) + Math.pow(kWheelBase / 2, 2));

    public static final double kDirectionSlewRate = 2.4; // radians per second
    public static final double kMagnitudeSlewRate = 3.6; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 4; // percent per second (1 = 100%)
  }

  // Slew rate filter variables for controlling lateral acceleration
  // and preventing excessive swerve wheel wear.
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;
  private final SlewRateLimiter m_magLimiter = new SlewRateLimiter(Constants.kMagnitudeSlewRate);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.kRotationalSlewRate);

  // All the swerve modules.
  // NOTE: in 2023 season, we had an error that caused the swerve module to
  // "twitch" periodically, so here is a reference in case it happens again
  // https://www.chiefdelphi.com/t/vmx-pi-can-spark-max-ids-1-timed-out-while-waiting-for-periodic-status-0/402177/8
  private final RevMAXSwerveModule m_frontLeft = new RevMAXSwerveModule(CANMapping.frontLeftDrivingMotor,
      CANMapping.frontLeftTurningMotor, Constants.kFrontLeftChassisAngularOffset);

  private final RevMAXSwerveModule m_frontRight = new RevMAXSwerveModule(CANMapping.frontRightDrivingMotor,
      CANMapping.frontRightTurningMotor, Constants.kFrontRightChassisAngularOffset);

  private final RevMAXSwerveModule m_rearLeft = new RevMAXSwerveModule(CANMapping.rearLeftDrivingMotor,
      CANMapping.rearLeftTurningMotor, Constants.kRearLeftChassisAngularOffset);

  private final RevMAXSwerveModule m_rearRight = new RevMAXSwerveModule(CANMapping.rearRightDrivingMotor,
      CANMapping.rearRightTurningMotor, Constants.kRearRightChassisAngularOffset);

  // Kinematics for the swerve drive.
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.kWheelBase / 2, Constants.kTrackWidth / 2),
      new Translation2d(Constants.kWheelBase / 2, -Constants.kTrackWidth / 2),
      new Translation2d(-Constants.kWheelBase / 2, Constants.kTrackWidth / 2),
      new Translation2d(-Constants.kWheelBase / 2, -Constants.kTrackWidth / 2));

  // Tracking robot pose.
  // TODO: try SwerveDrivePoseEstimator later this season
  private final WPI_Pigeon2 m_gyro = new WPI_Pigeon2(CANMapping.gyroSensor);
  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      m_kinematics,
      getGyroAngle(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });
  private final Field2d m_field = new Field2d();

  public Drivetrain() {
    SmartDashboard.putData("Field", m_field);
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(Constants.autoTranslationP, Constants.autoTranslationI, Constants.autoTranslationD),
            new PIDConstants(Constants.autoRotationP, Constants.autoRotationI, Constants.autoRotationD),
            Constants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
            Constants.kDriveBaseRadiusInMeters, // Distance from robot center to furthest module.
            new ReplanningConfig(false, false) // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    m_odometry.update(
        getGyroAngle(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    m_field.setRobotPose(getPose());
  }

  ChassisSpeeds getRobotRelativeSpeeds() {
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    };
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void resetPose(Pose2d pose) {
    m_odometry.resetPosition(
        getGyroAngle(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Resets the gyro as if the robot were facing away from you.
   * This is helpful for resetting field-oriented driving.
   */
  public void recalibrateDrivetrain() {
    double recalibratedAngleRadians = 0;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      recalibratedAngleRadians = Math.PI;
    }
    var currentPose = getPose();
    var currentTranslation = currentPose.getTranslation();
    var recalibratedRotation = new Rotation2d(recalibratedAngleRadians);
    var recalibratedPose = new Pose2d(currentTranslation, recalibratedRotation);
    resetPose(recalibratedPose);
  }

  void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds, false);
  }

  // Reference:
  // https://github.com/firebears-frc/FB2024/blob/main/src/main/java/frc/robot/subsystems/Bass.java#L284
  void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    if (fieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation());
    }
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      xSpeed = -1 * xSpeed;
      ySpeed = -1 * ySpeed;
    }

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(Constants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality
                                              // checking
                                              // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * Constants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * Constants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * Constants.kMaxAngularSpeed;

    drive(new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered), fieldRelative);

  }

  public void stop() {
    drive(0, 0, 0, true, true);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setXFormation() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  Rotation2d getGyroAngle() {
    return Rotation2d.fromDegrees(m_gyro.getAngle() % 360 * -1);
  }

}