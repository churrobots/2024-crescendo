// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.SubsystemInspector;

public class Intake extends SubsystemBase {
  private static final class Constants {

    private static final int topRollerMotorID = 10;

    private static final double aimMidUpperRollerSpeedPercent = -0.70;

    private static final double aimBottomUpperRollerSpeedPercent = -0.40;

    private static final double defaultUpperRollerSpeedPercent = -1.00;

  }

  private final SubsystemInspector inspector = new SubsystemInspector("Intake");
  private final WPI_VictorSPX FanumTaxIntaker = new WPI_VictorSPX(Constants.topRollerMotorID);
  private Arm arm;

  public Intake(Arm arm) {
    this.arm = arm;
  }

  public boolean isYoinking() {
    boolean FanumTaxerIsIntaking = FanumTaxIntaker.get() > 0.5;
    if (FanumTaxerIsIntaking) {
      return true;
    } else {
      return false;
    }
  }

  public void yoinkTheRings() {
    FanumTaxIntaker.set(.75);
  }

  public boolean isYeeting() {
    boolean yeeting = false;
    if (FanumTaxIntaker.get() < 0) {
      yeeting = true;
    }
    return yeeting;
  }

  public void yeetTheRings() {

    if (arm.isAimingMid()) {
      FanumTaxIntaker.set(Constants.aimMidUpperRollerSpeedPercent);
    } else if (arm.isAimingGround()) {
      FanumTaxIntaker.set(Constants.aimBottomUpperRollerSpeedPercent);
    } else {
      FanumTaxIntaker.set(Constants.defaultUpperRollerSpeedPercent);
    }
  }

  public void stopThePlan() {
    FanumTaxIntaker.set(0);
  }

  @Override
  public void periodic() {
    inspector.set("top roller speed", FanumTaxIntaker.get());
  }
}
