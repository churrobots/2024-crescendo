// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.FalconUtils;
import frc.robot.helpers.SubsystemInspector;
import frc.robot.helpers.Tunables.TunableDouble;
import frc.robot.helpers.Tunables.TunableInteger;

public class Arm extends SubsystemBase {

  enum Level {
    LOW,
    MID,
    RESTING
  }

  private static final class Constants {

    private static final int armCanID = 12;

    private static final double calibrationVelocitySensorUnitsPerSecond = -3000;

    private static final int offsetMaxCounts = 1000;

    private static final int restingCounts = 3000;
    private static final int aimBottomCounts = 12400;
    private static final int aimMidCounts = 6500;
    private static final int receiveFromSubstationCounts = 8000;
    private static final int receiveFromGroundCounts = 18000;

    public static final TunableDouble kP = new TunableDouble("kP", 0.04);
    public static final TunableDouble kF = new TunableDouble("kF", 0.0); // 0.05
    public static final TunableDouble kI = new TunableDouble("kI", 0.0); // 0.000001
    public static final TunableDouble kD = new TunableDouble("kD", 0.0);

    public static final TunableDouble kMaxGravityFeedForward = new TunableDouble("kMaxGravityFeedForward", -0.07);
    public static final TunableInteger kArmSpeed = new TunableInteger("kArmSpeed", 6000);
    public static final TunableInteger kArmAcceleration = new TunableInteger("kArmAcceleration", 75000);
    public static final TunableInteger kArmSmoothing = new TunableInteger("kArmSmoothing", 1);

  }

  private final SubsystemInspector m_inspector = new SubsystemInspector(getSubsystem());
  private Level level = Level.RESTING;

  private final WPI_TalonFX armMotor = new WPI_TalonFX(Constants.armCanID);
  private boolean m_isCalibrated = false;

  public Arm() {
    FalconUtils.initializeMotorWithConsistentSettings(armMotor, NeutralMode.Brake);
    updateArmTuning();
  }

  private double calculateFeedForward() {
    int kMeasuredPosHorizontal = 22673; // Position measured when arm is horizontal
    double kTicksPerDegree = 53828 / 360; // Sensor is 1:1 with arm rotation
    double currentPos = armMotor.getSelectedSensorPosition();
    double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);
    double maxGravityFF = Constants.kMaxGravityFeedForward.get();
    return maxGravityFF * cosineScalar;
  }

  private void runMotorWithSafety(TalonFXControlMode mode, double value) {
    if (m_isCalibrated) {
      if (mode == TalonFXControlMode.MotionMagic) {
        if (value >= Constants.aimMidCounts - Constants.offsetMaxCounts
            && value <= Constants.aimMidCounts + Constants.offsetMaxCounts) {
          level = Level.MID;
        } else if (value > Constants.aimMidCounts + Constants.offsetMaxCounts) {
          level = Level.LOW;
        } else {
          level = Level.RESTING;
        }
        armMotor.set(mode, value, DemandType.ArbitraryFeedForward, calculateFeedForward());
        m_inspector.set("target", value);
        m_inspector.set("actual", armMotor.getSelectedSensorPosition());
      } else {
        armMotor.set(mode, value);
      }
    }
  }

  public boolean isAimingMid() {
    return level == Level.MID;
  }

  public boolean isAimingGround() {
    return level == Level.LOW;
  }

  public boolean isResting() {
    return level == Level.RESTING;
  }

  public void resetCalibration() {
    armMotor.setSelectedSensorPosition(0);
    m_isCalibrated = true;
  }

  public void moveIntoCalibrationPosition() {
    m_isCalibrated = false;
    armMotor.set(TalonFXControlMode.Velocity, Constants.calibrationVelocitySensorUnitsPerSecond);
  }

  public void receiveFromSingleSubstation(double offset) {
    offset *= Constants.offsetMaxCounts;
    runMotorWithSafety(TalonFXControlMode.MotionMagic, Constants.receiveFromSubstationCounts + offset);
  }

  public void receiveFromGround() {
    var gravityWillTakeItTheRestOfTheWay = armMotor.getSelectedSensorPosition() > Constants.receiveFromGroundCounts;
    if (gravityWillTakeItTheRestOfTheWay) {
      armMotor.set(TalonFXControlMode.PercentOutput, 0);
    } else {
      runMotorWithSafety(TalonFXControlMode.MotionMagic, 1.15 * Constants.receiveFromGroundCounts);
    }
  }

  public void moveToLow() {
    runMotorWithSafety(TalonFXControlMode.MotionMagic, Constants.aimBottomCounts);
  }

  public void moveToLow(double offset) {
    offset *= Constants.offsetMaxCounts;
    runMotorWithSafety(TalonFXControlMode.MotionMagic, Constants.aimBottomCounts + offset);
  }

  public void moveToMid() {
    runMotorWithSafety(TalonFXControlMode.MotionMagic, Constants.aimMidCounts);
  }

  public void moveToMid(double offset) {
    offset *= Constants.offsetMaxCounts;
    runMotorWithSafety(TalonFXControlMode.MotionMagic, Constants.aimMidCounts + offset);
  }

  public void restTheArm() {
    var gravityWillTakeItTheRestOfTheWay = armMotor.getSelectedSensorPosition() < Constants.restingCounts;
    if (gravityWillTakeItTheRestOfTheWay) {
      armMotor.set(TalonFXControlMode.PercentOutput, 0);
    } else {
      runMotorWithSafety(TalonFXControlMode.MotionMagic, 0.70 * Constants.restingCounts);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    boolean armTuningChanged = Constants.kArmSpeed.didChange() ||
        Constants.kArmAcceleration.didChange() ||
        Constants.kArmSmoothing.didChange() ||
        Constants.kP.didChange() ||
        Constants.kF.didChange() ||
        Constants.kI.didChange() ||
        Constants.kD.didChange();

    if (armTuningChanged) {
      updateArmTuning();
    }
  }

  private void updateArmTuning() {
    FalconUtils.configureMotionMagic(
        armMotor,
        Constants.kArmSpeed.get(),
        Constants.kArmAcceleration.get(),
        Constants.kArmSmoothing.get(),
        Constants.kP.get(),
        Constants.kF.get(),
        Constants.kI.get(),
        Constants.kD.get());
  }

}
