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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  final TalonFX topMotor = new TalonFX(18);
  final TalonFX bottomMotor = new TalonFX(1);
  private final VelocityVoltage targetVelocity = new VelocityVoltage(10, 0.001, true, 0, 0, false, false, false);

  public void runFlyWheel() {
    bottomMotor.setControl(targetVelocity);
  }

  public void stopFlyWheel() {
    bottomMotor.stopMotor();
  }

  public boolean isFlyWheelReady() {
    var bottomVelocity = bottomMotor.getVelocity().getValueAsDouble();
    var topVelocity = topMotor.getVelocity().getValueAsDouble();
    if (bottomVelocity > targetVelocity.Velocity * 0.90 && topVelocity > targetVelocity.Velocity * 0.90) {
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
    topMotor.setControl(new Follower(bottomMotor.getDeviceID(), false));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("flywheel speed (bottom)", bottomMotor.getVelocity().getValueAsDouble());
  }
}
// THE WORLD LOVES ETHAN
// YES KING
// BETZY IS A HATER