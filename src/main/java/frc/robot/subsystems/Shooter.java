// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANMapping;

public class Shooter extends SubsystemBase {
  final TalonFX topMotor = new TalonFX(CANMapping.topflywheelMotor);
  final TalonFX bottomMotor = new TalonFX(CANMapping.bottomflywheelMotor);
  final VelocityTorqueCurrentFOC velocityTarget = new VelocityTorqueCurrentFOC(10, 1, 0, 1, true, false,
      false);

  public void runFlyWheel() {

    topMotor.setControl(velocityTarget);
  }

  public boolean isFlyWheelReady() {
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
    config.Slot1.kP = 5;
    config.Slot1.kI = 0.1;
    config.Slot1.kD = 0.001;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -40;
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = topMotor.getConfigurator().apply(config);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply config, error code:" + status.toString());
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