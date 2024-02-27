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
import frc.robot.helpers.Tunables.TunableBoolean;
import frc.robot.helpers.Tunables.TunableDouble;

public class Shooter extends SubsystemBase {

  final TalonFX topMotor = new TalonFX(CANMapping.topflywheelMotor);
  final TalonFX bottomMotor = new TalonFX(CANMapping.bottomflywheelMotor);
  // 40, 29 is good but slow
  final VelocityVoltage topvelocityTarget = new VelocityVoltage(47, 0.001, true, 0, 0, false, false, false);
  final VelocityVoltage bottomvelocityTarget = new VelocityVoltage(30, 0.001, true, 0, 0, false, false, false);
  final VelocityVoltage ampYeetTarget = new VelocityVoltage(0, 0.001, true, 0, 0, false, false, false);
  final VelocityVoltage speakerYeetTarget = new VelocityVoltage(1, 0.001, true, 0, 0, false, false, false);
  final VelocityVoltage reverseAmpYeetTarget = new VelocityVoltage(-3, 0.001, true, 0, 0, false, false, false);

  public void runFlywheelForSpeaker() {
    topMotor.setControl(topvelocityTarget);
    bottomMotor.setControl(bottomvelocityTarget);
  }

  public boolean isFlyWheelReadyForSpeaker() {
    double tolerance = 1.5;
    var actualTopVelocity = topMotor.getVelocity().getValueAsDouble();
    var actualBottomVelocity = bottomMotor.getVelocity().getValueAsDouble();
    var expectedTopVelocity = topvelocityTarget.Velocity;
    var expectedBottomVelocity = bottomvelocityTarget.Velocity;
    if (actualTopVelocity > (expectedTopVelocity - tolerance)
        && actualTopVelocity < (expectedTopVelocity + tolerance)
        && actualBottomVelocity > (expectedBottomVelocity - tolerance)
        && actualBottomVelocity < (expectedBottomVelocity + tolerance)) {
      return true;
    } else {
      return false;
    }
  }

  public void runAmpYeeter() {

    topMotor.setControl(ampYeetTarget);
    bottomMotor.setControl(ampYeetTarget);
  }

  public void reverseAmpYeeter() {

    topMotor.setControl(reverseAmpYeetTarget);
    bottomMotor.setControl(reverseAmpYeetTarget);
  }

  public void runSpeakerYeeter() {

    topMotor.setControl(speakerYeetTarget);
    bottomMotor.setControl(speakerYeetTarget);
  }

  public void stopFlyWheel() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }

  // TODO: no longer valid now we treat top and bottom with separate velocities
  public boolean isFlyWheelReady(double targetVelocity) {
    double tolerance = 1.5;
    var topVelocity = topMotor.getVelocity().getValueAsDouble();
    var bottomVelocity = bottomMotor.getVelocity().getValueAsDouble();
    if (topVelocity > (targetVelocity - tolerance)
        && topVelocity < (targetVelocity + tolerance)
        && bottomVelocity > (targetVelocity - tolerance)
        && bottomVelocity < (targetVelocity + tolerance)) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isFlywheelAmpReady() {
    return isFlyWheelReady(ampYeetTarget.Velocity);
  }

  public boolean isFlywheelSpeakerReady() {
    return isFlyWheelReady(speakerYeetTarget.Velocity);
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
    // TODO: this doesn't seem to target the same velocity, might have to do it
    // manually?
    // bottomMotor.setControl(new Follower(topMotor.getDeviceID(), false));
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
// NU UH
// MATEOS AN OPP
// BETZY'S A SNITCH
// DANIEL IS SUS