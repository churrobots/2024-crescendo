// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class CANMapping {
  public static final int armCan = 21;
  public static final int climberCan = 10;
  public static final int intakecan = 11;
  public static final int topflywheelcan = 18;
  public static final int bottomflywheelcan = 1;
  public static final int kFrontLeftDrivingCanId = 5;
  public static final int kRearLeftDrivingCanId = 7;
  public static final int kFrontRightDrivingCanId = 6;
  public static final int kRearRightDrivingCanId = 8;

  // NOTE: this error caused the swerve module to "twitch" periodically
  // https://www.chiefdelphi.com/t/vmx-pi-can-spark-max-ids-1-timed-out-while-waiting-for-periodic-status-0/402177/8
  public static final int kFrontLeftTurningCanId = 1;
  public static final int kRearLeftTurningCanId = 3;
  public static final int kFrontRightTurningCanId = 2;
  public static final int kRearRightTurningCanId = 4;

  // Gyro config
  public static final int kGyroCanId = 9;
}
