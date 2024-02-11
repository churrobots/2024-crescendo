// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private static final class Constants {

    private static final int rollerCanID = 11;

  }

  private final WPI_VictorSPX fanumTaxIntaker = new WPI_VictorSPX(Constants.rollerCanID);

  public Intake() {
    fanumTaxIntaker.setInverted(true);
  }

  public boolean isYoinking() {
    boolean fanumTaxerIsIntaking = fanumTaxIntaker.get() > 0.25;
    if (fanumTaxerIsIntaking) {
      return true;
    } else {
      return false;
    }
  }

  public void yoinkTheRings() {
    fanumTaxIntaker.set(.75);
  }

  public void deuceTheRings() {
    fanumTaxIntaker.set(-.40);
  }

  public boolean isDeucing() {
    boolean deucing = false;
    if (fanumTaxIntaker.get() < -0.25) {
      deucing = true;
    }
    return deucing;
  }

  public void stopThePlan() {
    fanumTaxIntaker.set(0);
  }

  @Override
  public void periodic() {
    // if you need anything ongoing
  }
}
