// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANMapping;

public class Shooter extends SubsystemBase {
  final TalonFX topMotor = new TalonFX(CANMapping.topflywheelMotor);
  final TalonFX bottomMotor = new TalonFX(CANMapping.bottomflywheelMotor);
  final VelocityVoltage velocityTarget = new VelocityVoltage(10, 0.001, true, 0, 0, false, false, false);
  final VelocityVoltage ampYeetTarget = new VelocityVoltage(10, 0.001, true, 0, 0, false, false, false);
  final VelocityVoltage speakerYeetTarget = new VelocityVoltage(10, 0.001, true, 0, 0, false, false, false);

  public void runFlyWheel() {

    topMotor.setControl(velocityTarget);
  }

  public void runAmpYeeter() {

    topMotor.setControl(ampYeetTarget);
  }

  public void runSpeakerYeeter() {

    topMotor.setControl(speakerYeetTarget);
  }

  public void stopFlyWheel() {
    topMotor.stopMotor();
  }

  // TODO: Update these to use the constants above for the speeds
  public boolean isFlyWheelReady() {
    var topVelocity = topMotor.getVelocity().getValueAsDouble();
    var bottomVelocity = bottomMotor.getVelocity().getValueAsDouble();
    if (topVelocity > 9.0 && bottomVelocity > 9.0) {
      return true;
    } else {
      return false;
    }
  }

  // TODO: Update these to use the constants above for the speeds
  public boolean FlywheelAmpReady() {
    var topVelocity = topMotor.getVelocity().getValueAsDouble();
    var bottomVelocity = bottomMotor.getVelocity().getValueAsDouble();
    if (topVelocity > 9.0 && bottomVelocity > 9.0) {
      return true;
    } else {
      return false;
    }
  }

  // TODO: Update these to use the constants above for the speeds
  public boolean FlywheelSpeakerReady() {
    var topVelocity = topMotor.getVelocity().getValueAsDouble();
    var bottomVelocity = bottomMotor.getVelocity().getValueAsDouble();
    if (topVelocity > 9.0 && bottomVelocity > 9.0) {
      return true;
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
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

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
    bottomMotor.setControl(new Follower(topMotor.getDeviceID(), false));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
// THE WORLD LOVES ETHAN
// YES KING
// BETZY IS A HATER
// DANIEL IS OFF THE MEDS