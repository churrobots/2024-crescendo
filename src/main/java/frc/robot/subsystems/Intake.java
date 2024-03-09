// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANMapping;

public class Intake extends SubsystemBase {

  private final WPI_VictorSPX fanumTaxIntaker = new WPI_VictorSPX(CANMapping.intakeMotor);

  public Intake() {
    fanumTaxIntaker.setInverted(true);
  }

  public boolean isYoinking() {
    boolean fanumTaxerIsIntaking = fanumTaxIntaker.get() > 0.5;
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
    fanumTaxIntaker.set(-.35);
  }

  public boolean isDeucing() {
    boolean deucing = false;
    if (fanumTaxIntaker.get() < 0) {
      deucing = true;
    }
    return deucing;
  }

  public void ejectNow() {
    fanumTaxIntaker.set(-.85);
  }

  public void stopThePlan() {
    fanumTaxIntaker.set(0);
  }

  @Override
  public void periodic() {
    // if you need anything ongoing
  }
}
