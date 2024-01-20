// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.helpers;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * Set of helpers for Falcon 500 motors.
 */
public class FalconUtils {

  /**
   * Set up a Falcon500 for MotionMagic. After you call this on the motor, you
   * should be able to set a MotionMagic position like this:
   * 
   * <pre>
   * myMotor.set(TalonFXControlMode.MotionMagic, 10000);
   * </pre>
   * 
   * @param motor
   * @param velocitySensorUnitsPerSecond
   * @param accelerationSensorUnitsPer100msPerSec
   * @param sCurveStrengthBetween0and8            0 is a trapezoid, 8 is a full
   *                                              s-curve
   * @param kP
   * @param kF
   * @param kI
   * @param kD
   */
  public static void configureMotionMagic(
      final WPI_TalonFX motor,
      final int velocitySensorUnitsPerSecond,
      final int accelerationSensorUnitsPer100msPerSec,
      final int sCurveStrengthBetween0and8,
      final double kP, final double kF, final double kI, final double kD) {

    // This sets a lot of the defaults that the example code seems to require
    // for full functioning of the Falcon500s. Cargo culting FTW.
    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/MotionMagic/src/main/java/frc/robot/Robot.java
    // TODO: why would you ever not use the 0th slots?
    int profileSlotThatSeemsToWork = 0;
    int pidSlotThatSeemsToWork = 0;
    double peakOutputThatSeemsToWork = 0.4;
    int timoutThatSeemsToWorkInMilliseconds = 30;
    int periodMillisecondsThatDoesntOveruseCanBus = 20;

    // TODO: is tiny deadzone necessary for MotionMagic? or just for example?
    motor.configNeutralDeadband(0.001);

    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,
        periodMillisecondsThatDoesntOveruseCanBus, timoutThatSeemsToWorkInMilliseconds);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,
        periodMillisecondsThatDoesntOveruseCanBus, timoutThatSeemsToWorkInMilliseconds);

    motor.selectProfileSlot(profileSlotThatSeemsToWork, pidSlotThatSeemsToWork);
    motor.config_kF(profileSlotThatSeemsToWork, kF, timoutThatSeemsToWorkInMilliseconds);
    motor.config_kP(profileSlotThatSeemsToWork, kP, timoutThatSeemsToWorkInMilliseconds);
    motor.config_kI(profileSlotThatSeemsToWork, kI, timoutThatSeemsToWorkInMilliseconds);
    motor.config_kD(profileSlotThatSeemsToWork, kD, timoutThatSeemsToWorkInMilliseconds);

    motor.configMotionCruiseVelocity(velocitySensorUnitsPerSecond, timoutThatSeemsToWorkInMilliseconds);
    motor.configMotionAcceleration(accelerationSensorUnitsPer100msPerSec, timoutThatSeemsToWorkInMilliseconds);
    motor.configMotionSCurveStrength(sCurveStrengthBetween0and8);

    motor.configPeakOutputForward(peakOutputThatSeemsToWork);
    motor.configPeakOutputReverse(-1 * peakOutputThatSeemsToWork);
  }

  /**
   * Initialize a Falcon500 to factory defaults with a consistent brake mode.
   * Also attempt to set safe current limits according to their docs:
   * https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/d70cab6060617bbed5e207c2eaf8747af09a15f6/Java%20Talon%20FX%20(Falcon%20500)/Current%20Limit/src/main/java/frc/robot/Robot.java#L82-L92
   */
  public static void initializeMotorWithConsistentSettings(WPI_TalonFX motor, NeutralMode neutralMode) {
    // TODO: migrate to Phoenix 6
    // https://v6.docs.ctr-electronics.com/en/stable/docs/migration/migration-guide/feature-replacements-guide.html
    motor.configFactoryDefault();
    motor.setNeutralMode(neutralMode);
    motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5));
  }

}
