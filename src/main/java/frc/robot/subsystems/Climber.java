// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.FalconUtils;

public class Climber extends SubsystemBase {

  final TalonFX climberMotor = new TalonFX(10);

  public Climber() {
    FalconUtils.initializeMotorWithConsistentSettings(climberMotor, NeutralMode.Brake);
  }

  double getEncoderCounts() {
    return climberMotor.getPosition().getValueAsDouble();
  }

  public void goUp() {
    var isAtTheTop = getEncoderCounts() >= 9000;
    if (isAtTheTop) {
      stay();
    } else {
      climberMotor.set(0.5);
    }
  }

  public void goDown() {
    var isAtTheBottom = getEncoderCounts() >= -9000;
    if (isAtTheBottom) {
      stay();
    } else {
      climberMotor.set(-0.5);
    }
  }

  public void stay() {
    climberMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
