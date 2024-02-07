// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.helpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

/**
 * This is a helper for setting specific structured data that is visible
 * in AdvantageScope by WPILib.
 */
public class AdvantageScopeUtils {

  // https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/docs/tabs/ODOMETRY.md#structured-format
  public static StructPublisher<Pose2d> robotPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("Churrobots/RobotPose", Pose2d.struct).publish();

  public static void logRobotPose(Pose2d pose) {
    robotPosePublisher.set(pose);
  }
}
