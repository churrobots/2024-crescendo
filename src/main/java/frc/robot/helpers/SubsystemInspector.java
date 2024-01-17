// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.helpers;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SubsystemInspector {

  private final String rootName;

  public SubsystemInspector(String subsystemName) {
    rootName = subsystemName;
  }

  public void set(String name, Boolean value) {
    _getEntry(name).setBoolean(value);
  }

  private NetworkTableEntry _getEntry(String name) {
    return SmartDashboard.getEntry(rootName + "." + name);
  }

  public void set(String name, String value) {
    _getEntry(name).setString(value);
  }

  public void set(String name, Double value) {
    boolean differenceIsSignificantEnoughToUpdateEntry = Math
        .abs(value - _getEntry(name).getDouble(Double.MAX_VALUE)) > 0.01;
    if (differenceIsSignificantEnoughToUpdateEntry) {
      _getEntry(name).setDouble(value);
    }
  }

  public void set(String name, Integer value) {
    _getEntry(name).setDouble(value);
  }

  public void set(String name, Command value) {
    _getEntry(name).setString(value != null ? value.getName() : "---");
  }

}
