// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class YahtzeeBalance extends CommandBase {

  private final Drivetrain m_drivetrain;

  public YahtzeeBalance(Drivetrain drivetrain) {
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    var angle = m_drivetrain.getPitch();
    if (angle < -6.00) {
      m_drivetrain.drive(0.05, 0.0, 0, false, true);
    } else if (angle > 6.00) {
      m_drivetrain.drive(-0.05, 0.0, 0, false, true);
    } else {
      m_drivetrain.setXFormation();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TODO: make sure to stop the drivetrain at this point
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
