// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.helpers;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

/** Add your docs here. */
public class LogitechX3D extends GenericHID {

  public LogitechX3D(final int port) {
    super(port);
    HAL.report(tResourceType.kResourceType_Joystick, port + 1);
  }

  public double getY() {
    return getRawAxis(1);
    // return getRawAxis(20);
  }

  public double getX() {
    return getRawAxis(0);
    // return getRawAxis(21);
  }

  public double getTwist() {
    return getRawAxis(2);
    // return getRawAxis(24);
  }

  public double getThrottle() {
    return getRawAxis(25);
  }
  // Button Pressing values

}
