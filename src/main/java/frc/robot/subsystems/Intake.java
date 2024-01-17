// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private static final class Constants {

    private static final int topRollerMotorID = 10;
    private static final int bottomRollerMotorID = 11;

    private static final double aimMidUpperRollerSpeedPercent = -0.70;
    private static final double aimMidLowerRollerSpeedPercent = -1.00;

    private static final double aimBottomUpperRollerSpeedPercent = -0.40;
    private static final double aimBottomLowerRollerSpeedPercent = -0.35;
    private static final double defaultUpperRollerSpeedPercent = -1.00;
    private static final double defaultLowerRollerSpeedPercent = -1.00;

  }

  private final WPI_VictorSPX topCubeYoinker = new WPI_VictorSPX(Constants.topRollerMotorID);
  private final WPI_VictorSPX bottomCubeYoinker = new WPI_VictorSPX(Constants.bottomRollerMotorID);
  private Arm arm;

  public Intake(Arm arm) {
    this.arm = arm;
  }

  public boolean isYoinking() {
    boolean yoinking = false;
    if (topCubeYoinker.get() > 0.5 && bottomCubeYoinker.get() > 0.5) {
      yoinking = true;
    }
    return yoinking;
  }

  public void yoinkTheCubes() {
    topCubeYoinker.set(.75);
    bottomCubeYoinker.set(.75);
  }
  
  public boolean isYeeting() {
    boolean yeeting = false;
    if (topCubeYoinker.get() < 0 && bottomCubeYoinker.get() < 0) {
      yeeting = true;
    }
    return yeeting;
  }
  public void yeetTheCubes() {

    if (arm.isAimingMid()) {
      topCubeYoinker.set(Constants.aimMidUpperRollerSpeedPercent);
      bottomCubeYoinker.set(Constants.aimMidLowerRollerSpeedPercent);
    } else if (arm.isAimingGround()) {
      topCubeYoinker.set(Constants.aimBottomUpperRollerSpeedPercent);
      bottomCubeYoinker.set(Constants.aimBottomLowerRollerSpeedPercent);
    } else {
      topCubeYoinker.set(Constants.defaultUpperRollerSpeedPercent);
      bottomCubeYoinker.set(Constants.defaultLowerRollerSpeedPercent);
    }
  }

  public void stopThePlan() {
    topCubeYoinker.set(0);
    bottomCubeYoinker.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
