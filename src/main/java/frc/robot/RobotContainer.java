// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LightShow;

public class RobotContainer {

  static final class Constants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorrControllerPort = 1;
    public static final double kDriveDeadband = 0.1;
    public static final double slowDriveScaling = 0.4;
  }

  // Drive and Operator interface.
  final XboxController m_driverController = new XboxController(Constants.kDriverControllerPort);
  final XboxController m_operatorController = new XboxController(Constants.kOperatorrControllerPort);
  SendableChooser<Command> autoChooser;

  // All of the subsystems.
  final Drivetrain m_drivetrain = new Drivetrain();
  final Arm m_arm = new Arm();
  final Intake m_intake = new Intake(m_arm);
  final LightShow m_lightShow = new LightShow();

  // Commands the robot can do.
  final Command doNothing = Commands.none();

  final Command yeetFar = new RunCommand(m_lightShow::setRed, m_lightShow);
  final Command yeetClose = new RunCommand(m_lightShow::setYellow, m_lightShow);
  final Command yoinkNote = new RunCommand(m_lightShow::setBlue, m_lightShow);

  final Command anchorInPlace = new RunCommand(() -> m_drivetrain.setXFormation(), m_drivetrain);
  final Command resetGyro = new RunCommand(() -> m_drivetrain.resetGyro(), m_drivetrain);

  final Command showPurple = new RunCommand(m_lightShow::setPurple, m_lightShow);

  // Sensors for robot states.
  final BooleanSupplier isShooting = () -> m_intake.isYeeting();

  final Command slowDrive = new RunCommand(
      () -> m_drivetrain.drive(
          -MathUtil.applyDeadband(m_driverController.getLeftY() * Constants.slowDriveScaling,
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getLeftX() * Constants.slowDriveScaling,
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getRightX() * Constants.slowDriveScaling,
              Constants.kDriveDeadband),
          true, true),
      m_drivetrain);

  final Command fastDrive = new RunCommand(
      () -> m_drivetrain.drive(
          -MathUtil.applyDeadband(m_driverController.getLeftY(),
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getLeftX(),
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getRightX(),
              Constants.kDriveDeadband),
          true, true),
      m_drivetrain);

  public RobotContainer() {
    configureButtonBindings();
    ensureSubsystemsHaveDefaultCommands();
    createAutonomousSelector();
  }

  /**
   * This is called by the system when automomous runs, and it
   * should return the command you want to execute when automous
   * mode begins.
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void createAutonomousSelector() {
    NamedCommands.registerCommand("yeetClose", yeetClose);
    NamedCommands.registerCommand("yeetFar", yeetFar.withTimeout(4));
    NamedCommands.registerCommand("yoinkNote", yoinkNote.withTimeout(4));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
  }

  void configureButtonBindings() {

    // Driver
    var startButton = new JoystickButton(m_driverController, Button.kStart.value);
    var leftBumper = new JoystickButton(m_driverController, Button.kLeftBumper.value);
    var rightBumper = new JoystickButton(m_driverController, Button.kRightBumper.value);

    leftBumper.whileTrue(anchorInPlace);
    rightBumper.whileTrue(slowDrive);
    startButton.whileTrue(resetGyro);

    // Operator
    var leftBumperOpButton = new JoystickButton(m_operatorController, Button.kLeftBumper.value);
    var rightBumperOpButton = new JoystickButton(m_operatorController, Button.kRightBumper.value);
    var aOpButton = new JoystickButton(m_operatorController, Button.kA.value);
    var xOpButton = new JoystickButton(m_operatorController, Button.kX.value);
    var yOpButton = new JoystickButton(m_operatorController, Button.kY.value);
    var bOpButton = new JoystickButton(m_operatorController, Button.kB.value);
    var startOpButton = new JoystickButton(m_operatorController, Button.kStart.value);
    var backOpButton = new JoystickButton(m_operatorController, Button.kBack.value);

    // TODO: wire up commands to the operator buttons
    leftBumperOpButton.whileTrue(doNothing);
    rightBumperOpButton.whileTrue(doNothing);
    backOpButton.whileTrue(doNothing);
    startOpButton.whileTrue(doNothing);
    xOpButton.whileTrue(doNothing);
    aOpButton.whileTrue(doNothing);
    yOpButton.whileTrue(doNothing);
    bOpButton.whileTrue(doNothing);
  }

  void ensureSubsystemsHaveDefaultCommands() {
    // TODO: set up default commands for these
    m_arm.setDefaultCommand(doNothing);
    m_intake.setDefaultCommand(doNothing);
    m_drivetrain.setDefaultCommand(fastDrive);
    m_lightShow.setDefaultCommand(showPurple);
    m_lightShow.setPurple(); // manually init color since default commands don't run until robot is enabled
  }

}