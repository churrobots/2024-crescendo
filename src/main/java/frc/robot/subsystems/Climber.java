// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANMapping;
import frc.robot.helpers.FalconUtils;

public class Climber extends SubsystemBase {

  final WPI_TalonFX climberMotor = new WPI_TalonFX(CANMapping.climberMotor);

  public Climber() {
    FalconUtils.initializeMotorWithConsistentSettings(climberMotor, NeutralMode.Brake);
    climberMotor.setSelectedSensorPosition(0);
  }

  public void goUp() {
    var isAtTheTop = climberMotor.getSelectedSensorPosition() <= 210000;
    if (isAtTheTop) {
      stay();
    } else {
      climberMotor.set(.9);
    }
  }

  public void goDown() {
    var isAtTheBottom = climberMotor.getSelectedSensorPosition() >= 0;
    if (isAtTheBottom) {
      stay();
    } else {
      climberMotor.set(-0.9);
    }
  }

  public void stay() {
    climberMotor.set(0);
  }

  public void goUpWNoSafety() {
    climberMotor.set(0.5);

  }

  public void goDownWNoSafety() {
    climberMotor.set(-0.5);
    climberMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("climber:position", climberMotor.getSelectedSensorPosition());
  }
}
