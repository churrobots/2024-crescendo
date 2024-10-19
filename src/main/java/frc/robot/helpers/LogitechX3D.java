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

  // front-back
  public double getY() {
    return getRawAxis(1);
  }

  // left-right
  public double getX() {
    return getRawAxis(0);
  }

  // twist
  public double getTwist() {
    return getRawAxis(2);
  }

  // trim potentiometer
  public double getThrottle() {
    return getRawAxis(3);
  }

  // Button Pressing values
  

}
