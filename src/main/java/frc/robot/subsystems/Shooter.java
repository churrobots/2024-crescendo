// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANMapping;

public class Shooter extends SubsystemBase {

  public static final class Constants {
    public static final double targetVelocityTolerance = 1.5;

    public static final double topSpeakerVelocity = 50;
    public static final double bottomSpeakerVelocity = 50;

    public static final double ampVelocity = 7;
    public static final double ejectVelocity = -3;

    public static final double butterDusterVelocity = 15;
  }

  // Daniel does not have churrobot spirit
  final TalonFX topMotor = new TalonFX(CANMapping.topflywheelMotor);
  final TalonFX bottomMotor = new TalonFX(CANMapping.bottomflywheelMotor);

  final VelocityVoltage topSpeakerTarget = new VelocityVoltage(Constants.topSpeakerVelocity, 0.001, true, 0, 0, false,
      false, false);
  final VelocityVoltage buttDustTarget = new VelocityVoltage(Constants.butterDusterVelocity, 0.001, true, 0, 0, false,
      false, false);
  final VelocityVoltage bottomSpeakerTarget = new VelocityVoltage(Constants.bottomSpeakerVelocity, 0.001, true, 0, 0,
      false, false, false);

  final VelocityVoltage ampYeetTarget = new VelocityVoltage(Constants.ampVelocity, 0.001, true, 0, 0, false, false,
      false);
  final VelocityVoltage reverseAmpYeetTarget = new VelocityVoltage(Constants.ejectVelocity, 0.001, true, 0, 0, false,
      false, false);

  public void runFlywheelForSpeaker() {
    topMotor.setControl(topSpeakerTarget);
    bottomMotor.setControl(bottomSpeakerTarget);
  }

  public void runButtDustYeet() {
    topMotor.setControl(buttDustTarget);
    bottomMotor.setControl(buttDustTarget);
  }

  public void runAmpYeeter() {
    topMotor.setControl(ampYeetTarget);
  }

  public void reverseAmpYeeter() {
    topMotor.setControl(reverseAmpYeetTarget);
    bottomMotor.setControl(reverseAmpYeetTarget);
  }

  public void stopFlyWheel() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }

  public boolean flywheelsAreAtTarget() {
    return isMotorAtTarget(topMotor) && isMotorAtTarget(bottomMotor);
  }

  boolean isMotorAtTarget(TalonFX motor) {
    ControlRequest appliedControl = motor.getAppliedControl();
    boolean isNeutral = appliedControl.getName() == "NeutralOut";
    if (isNeutral) {
      return false;
    }
    if (appliedControl.getName() == "NeutralOut") {
      return true;
    } else if (appliedControl.getName() == "VelocityVoltage") {
      VelocityVoltage target = (VelocityVoltage) appliedControl;
      var actualVelocity = motor.getVelocity().getValueAsDouble();
      var expectedVelocity = target.Velocity;
      var minVelocity = expectedVelocity - Constants.targetVelocityTolerance;
      var maxVelocity = expectedVelocity + Constants.targetVelocityTolerance;
      if (actualVelocity > minVelocity && actualVelocity < maxVelocity) {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

  public Shooter() {
    var config = new TalonFXConfiguration();
    // TODO: consider adding these Phoenix6 equivalents of our old Phoenix5 config
    // config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // config.CurrentLimits.SupplyCurrentLimitEnable = true;
    // config.CurrentLimits.SupplyCurrentLimit = 10;
    // config.CurrentLimits.SupplyCurrentThreshold = 15;
    // config.CurrentLimits.SupplyTimeThreshold = 0.5;

    // These PID values suit a standard Falcon500, according to official example.
    // https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/6be53802b071f84fd45aeca23737345cfc421072/java/VelocityClosedLoop/src/main/java/frc/robot/Robot.java#L51-L54
    config.Slot0.kP = 0.11;
    config.Slot0.kI = 0.5;
    config.Slot0.kD = 0.0001;
    config.Slot0.kV = 0.12;
    config.Voltage.PeakForwardVoltage = 8;
    config.Voltage.PeakReverseVoltage = -8;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = bottomMotor.getConfigurator().apply(config);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      // TODO: set some sticky error state and show it via LEDs?
      System.out.println("Could not apply config, error code:" +
          status.toString());
    }
    for (int i = 0; i < 5; ++i) {
      status = topMotor.getConfigurator().apply(config);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      // TODO: set some sticky error state and show it via LEDs?
      System.out.println("Could not apply config, error code:" +
          status.toString());
    }
    // TODO: this doesn't seem to target the same velocity, might have to do it
    // manually?
    // bottomMotor.setControl(new Follower(topMotor.getDeviceID(), false));
  }

  @Override
  public void periodic() {
    var actualTopVelocity = topMotor.getVelocity().getValueAsDouble();
    var actualBottomVelocity = bottomMotor.getVelocity().getValueAsDouble();
    SmartDashboard.putNumber("topVelocity", actualTopVelocity);
    SmartDashboard.putNumber("bottomVelocity", actualBottomVelocity);
    // This method will be called once per scheduler run
  }
}
// THE WORLD LOVES ETHAN
// YES KING
// BETZY IS A HATER
// DANIEL IS OFF THE MEDS
// NU UH
// MATEOS AN OPP
// BETZY'S A SNITCH
// DANIEL IS SUS
// TATI IS THE BEST
// DANIEL IS BANNED FROM COMING