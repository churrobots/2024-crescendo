// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.helpers;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Tunables {

  protected abstract static class TunableEntry<T> {
    protected final T defaultValue;
    protected final NetworkTableEntry entry;
    protected long lastDetectedChange;

    public TunableEntry(String name, T defaultValue) {
      entry = SmartDashboard.getEntry(name);
      this.defaultValue = defaultValue;
      this._syncDefault();
      lastDetectedChange = entry.getLastChange();
    }

    abstract protected void _syncDefault();

    public boolean didChange() {
      boolean answer = lastDetectedChange != entry.getLastChange();
      lastDetectedChange = entry.getLastChange();
      return answer;
    }
  }

  public static class TunableBoolean extends TunableEntry<Boolean> {
    public TunableBoolean(String name, Boolean defaultValue) {
      super(name, defaultValue);
    }

    protected void _syncDefault() {
      entry.setBoolean(defaultValue);
    }

    public Boolean get() {
      return entry.getBoolean(defaultValue);
    }
  }

  public static class TunableDouble extends TunableEntry<Double> {
    public TunableDouble(String name, double d) {
      super(name, d);
    }

    protected void _syncDefault() {
      entry.setDouble(defaultValue);
    }

    public Double get() {
      return entry.getDouble(defaultValue);
    }
  }

  public static class TunableInteger extends TunableEntry<Integer> {
    public TunableInteger(String name, Integer defaultValue) {
      super(name, defaultValue);
    }

    protected void _syncDefault() {
      entry.setDouble(defaultValue);
    }

    public Integer get() {
      return (int) Math.floor(entry.getDouble(defaultValue));
    }
  }

}