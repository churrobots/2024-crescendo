// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.helpers.RevMAXSwerveModule;
import frc.robot.helpers.SubsystemInspector;
import frc.robot.helpers.SwerveUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private final RevMAXSwerveModule m_frontLeft;
  private final RevMAXSwerveModule m_frontRight;
  private final RevMAXSwerveModule m_rearLeft;
  private final RevMAXSwerveModule m_rearRight;
  public final WPI_Pigeon2 m_gyro;
  private final SwerveDriveKinematics m_kinematics;
  private final SlewRateLimiter m_magLimiter;
  private final SlewRateLimiter m_rotLimiter;
  private final double m_maxSpeedMetersPerSecond;
  private final double m_maxAngularSpeedRadiansPerSecond;
  private final double m_directionSlewRate;
  private final boolean m_gyroIsReversed;

  /**
   * Constants that work for the SpeedyHedgehog drive base.
   * This includes the CAN configuration as well as dimensions,
   * max speeds, etc that are particular to TurboSwervo.
   */
  private static final class SpeedyHedgehogConstants {

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(18.5);
    public static final double kWheelBase = Units.inchesToMeters(23.5);

    // Angular offsets of the modules relative to the chassis in radians
    // TODO: need to actually calculate these since the robot is not square
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kRearLeftChassisAngularOffset = Math.PI;
    public static final double kRearRightChassisAngularOffset = Math.PI / 2;

    // Autonomous: PID contants
    // TODO: we cribbed these from the Eastbots who have a maxswerve but we don't
    // know if it's comparable characteristics so we might want to reconsider, the
    // original values were all 1.0
    public static final double kPXController = 2;
    public static final double kPYController = 4;
    public static final double kPThetaController = 0.04;

    // Autonomous: Trajectory speed constants
    // FIXME: were both 3.0 before, but we slowed it down for balancing and safety
    public static final double kTrajectoryMaxSpeedMetersPerSecond = 2;
    public static final double kTrajectoryMaxAccelerationMetersPerSecondSquared = 0.8;

    // Note: at this point, we probably don't want to change these speeds too much
    // since we've tuned the PID values to work at these speeds.
    public static final double kTrajectoryMaxSpeedMetersPerSecondForAutoBuilder = 2.4;
    public static final double kTrajectoryMaxAccelerationMetersPerSecondSquaredForAutoBuilder = 1.0;

    // Note: these are for fast autos with events
    public static final double kFastTrajectoryMaxSpeedMetersPerSecondForAutoBuilder = 2.5;
    public static final double kFastTrajectoryMaxAccelerationMetersPerSecondSquaredForAutoBuilder = 1.7;

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 2.4; // radians per second
    public static final double kMagnitudeSlewRate = 3.6; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 4; // percent per second (1 = 100%)

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 5;
    public static final int kRearLeftDrivingCanId = 7;
    public static final int kFrontRightDrivingCanId = 6;
    public static final int kRearRightDrivingCanId = 8;

    public static final int kFrontLeftTurningCanId = 1;
    public static final int kRearLeftTurningCanId = 3;
    public static final int kFrontRightTurningCanId = 2;
    // TODO: we got this error one day, causes turning to "twitch" periodically
    // https://www.chiefdelphi.com/t/vmx-pi-can-spark-max-ids-1-timed-out-while-waiting-for-periodic-status-0/402177/8
    public static final int kRearRightTurningCanId = 4;

    // Gyro config
    public static final int kGyroCanId = 9;
    public static final boolean kGyroReversed = false;
  }

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;

  // Subsystem Inspector
  private final SubsystemInspector m_inspector = new SubsystemInspector(getSubsystem());

  public Drivetrain() {

    /////////////////////////////////////////////////////////////////
    // SpeedyHedgehog is a Rev MAXSwerve

    m_frontLeft = new RevMAXSwerveModule(
        SpeedyHedgehogConstants.kFrontLeftDrivingCanId,
        SpeedyHedgehogConstants.kFrontLeftTurningCanId,
        SpeedyHedgehogConstants.kFrontLeftChassisAngularOffset);

    m_frontRight = new RevMAXSwerveModule(
        SpeedyHedgehogConstants.kFrontRightDrivingCanId,
        SpeedyHedgehogConstants.kFrontRightTurningCanId,
        SpeedyHedgehogConstants.kFrontRightChassisAngularOffset);

    m_rearLeft = new RevMAXSwerveModule(
        SpeedyHedgehogConstants.kRearLeftDrivingCanId,
        SpeedyHedgehogConstants.kRearLeftTurningCanId,
        SpeedyHedgehogConstants.kRearLeftChassisAngularOffset);

    m_rearRight = new RevMAXSwerveModule(
        SpeedyHedgehogConstants.kRearRightDrivingCanId,
        SpeedyHedgehogConstants.kRearRightTurningCanId,
        SpeedyHedgehogConstants.kRearRightChassisAngularOffset);

    m_kinematics = new SwerveDriveKinematics(
        new Translation2d(SpeedyHedgehogConstants.kWheelBase / 2, SpeedyHedgehogConstants.kTrackWidth / 2),
        new Translation2d(SpeedyHedgehogConstants.kWheelBase / 2, -SpeedyHedgehogConstants.kTrackWidth / 2),
        new Translation2d(-SpeedyHedgehogConstants.kWheelBase / 2, SpeedyHedgehogConstants.kTrackWidth / 2),
        new Translation2d(-SpeedyHedgehogConstants.kWheelBase / 2, -SpeedyHedgehogConstants.kTrackWidth / 2));

    m_gyro = new WPI_Pigeon2(SpeedyHedgehogConstants.kGyroCanId);
    m_gyroIsReversed = SpeedyHedgehogConstants.kGyroReversed;

    m_magLimiter = new SlewRateLimiter(SpeedyHedgehogConstants.kMagnitudeSlewRate);
    m_rotLimiter = new SlewRateLimiter(SpeedyHedgehogConstants.kRotationalSlewRate);

    m_directionSlewRate = SpeedyHedgehogConstants.kDirectionSlewRate;
    m_maxAngularSpeedRadiansPerSecond = SpeedyHedgehogConstants.kMaxAngularSpeed;
    m_maxSpeedMetersPerSecond = SpeedyHedgehogConstants.kMaxSpeedMetersPerSecond;

    // Slew rate filter variables for controlling lateral acceleration
    m_currentRotation = 0.0;
    m_currentTranslationDir = 0.0;
    m_currentTranslationMag = 0.0;

    m_prevTime = WPIUtilJNI.now() * 1e-6;
    m_odometry = new SwerveDriveOdometry(
        m_kinematics,
        getGyroAngle(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  @Override
  public void periodic() {

    // Update the odometry in the periodic block
    m_odometry.update(
        getGyroAngle(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    m_inspector.set("Pitch", m_gyro.getPitch());
    m_inspector.set("Yaw", m_gyro.getYaw());
    m_inspector.set("Roll", m_gyro.getRoll());
    m_inspector.set("Angle", m_gyro.getAngle());
    m_inspector.set("Compass", m_gyro.getAbsoluteCompassHeading());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
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
  public void resetGyro() {
    m_gyro.reset();
  }

  public void resetGyroForAuto() {
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      m_gyro.setYaw(180);

    } else {
      m_gyro.setYaw(-180);
    }
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

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(m_directionSlewRate / m_currentTranslationMag);
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
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
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
    double xSpeedDelivered = xSpeedCommanded * m_maxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * m_maxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * m_maxAngularSpeedRadiansPerSecond;

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                getGyroAngle())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, m_maxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
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
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, m_maxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetDriveEncodersToZero();
    m_frontRight.resetDriveEncodersToZero();
    m_rearLeft.resetDriveEncodersToZero();
    m_rearRight.resetDriveEncodersToZero();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  public Rotation2d getGyroAngle() {
    return Rotation2d.fromDegrees(m_gyro.getAngle() % 360 * -1);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return getGyroAngle().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (m_gyroIsReversed ? -1.0 : 1.0);
  }

  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public void assertWheelsArePointedForwardAndStoreCalibration() {
    m_frontLeft.assertModuleIsPointedForwardAndStoreCalibration();
    m_frontRight.assertModuleIsPointedForwardAndStoreCalibration();
    m_rearLeft.assertModuleIsPointedForwardAndStoreCalibration();
    m_rearRight.assertModuleIsPointedForwardAndStoreCalibration();
  }

  // Assuming this method is part of a drivetrain subsystem that provides the
  // necessary methods
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if (isFirstPath) {
            this.resetPose(traj.getInitialHolonomicPose());
          }
        }),

        new PPSwerveControllerCommand(
            traj,
            this::getPose, // Pose supplier
            this.m_kinematics, // SwerveDriveKinematics
            new PIDController(
                SpeedyHedgehogConstants.kPXController,
                0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use
                       // feedforwards.
            new PIDController(
                SpeedyHedgehogConstants.kPYController,
                0, 0), // Y controller (usually the same values as
                       // X controller)
            new PIDController(
                SpeedyHedgehogConstants.kPThetaController,
                0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will
                       // only use feedforwards.
            this::setModuleStates, // Module states consumer
            false, // Should the path be automatically mirrored depending on alliance color.
                   // Optional, defaults to true
            this // Requires this drive subsystem
        ));
  }

  public double getPitch() {
    return m_gyro.getPitch();
  }

  public PathConstraints getPathPlannerConstraints() {
    return new PathConstraints(
        SpeedyHedgehogConstants.kTrajectoryMaxSpeedMetersPerSecond,
        SpeedyHedgehogConstants.kTrajectoryMaxAccelerationMetersPerSecondSquared);
  }

  public PathConstraints getPathPlannerConstraintsForAutoBuilder() {
    return new PathConstraints(
        SpeedyHedgehogConstants.kTrajectoryMaxSpeedMetersPerSecondForAutoBuilder,
        SpeedyHedgehogConstants.kTrajectoryMaxAccelerationMetersPerSecondSquaredForAutoBuilder);
  }

  public PathConstraints getPathPlannerConstraintsForFastAutoBuilder() {
    return new PathConstraints(
        SpeedyHedgehogConstants.kFastTrajectoryMaxSpeedMetersPerSecondForAutoBuilder,
        SpeedyHedgehogConstants.kFastTrajectoryMaxAccelerationMetersPerSecondSquaredForAutoBuilder);
  }
}