// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  // All of the commands the robot can do.
  final Command doNothing = Commands.none();

  final Command safelyRestTheArm = new RunCommand(m_arm::restTheArm, m_arm);
  final Command showBlue = new RunCommand(m_lightShow::setBlue, m_lightShow);
  final Command stopIntake = new RunCommand(m_intake::stopThePlan, m_intake);

  final Command anchorInPlace = new RunCommand(() -> m_drivetrain.setXFormation(), m_drivetrain);
  final Command resetGyro = new RunCommand(() -> m_drivetrain.resetGyro(), m_drivetrain);

  final Command yeet = new RunCommand(m_intake::yeetTheCubes, m_intake);
  final Command yoink = new RunCommand(m_intake::yoinkTheCubes, m_intake);

  final Command moveArmIntoCalibration = new RunCommand(m_arm::moveIntoCalibrationPosition, m_arm);
  final Command resetArmCalibration = new RunCommand(m_arm::resetCalibration, m_arm);
  final Command moveToReceive = new RunCommand(
      () -> m_arm.receiveFromSingleSubstation(-m_operatorController.getLeftY()),
      m_arm);
  final Command moveToLow = new RunCommand(() -> m_arm.moveToLow(-m_operatorController.getLeftY()), m_arm);
  final Command moveToMid = new RunCommand(() -> m_arm.moveToMid(-m_operatorController.getLeftY()), m_arm);
  final Command moveToGroundPickup = new RunCommand(m_arm::receiveFromGround, m_arm);

  final Command showPurple = new RunCommand(m_lightShow::setPurple, m_lightShow);

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

  public void handleDisable() {
    // TODO: we're not sure if we need this still
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
    NamedCommands.registerCommand("sayHello", showPurple);
    NamedCommands.registerCommand("sayBye", doNothing);
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

    leftBumperOpButton.whileTrue(yoink);
    rightBumperOpButton.whileTrue(yeet);
    backOpButton.whileTrue(moveArmIntoCalibration);
    startOpButton.whileTrue(resetArmCalibration);
    xOpButton.whileTrue(moveToLow);
    aOpButton.whileTrue(moveToMid);
    yOpButton.whileTrue(moveToReceive);
    bOpButton.whileTrue(moveToGroundPickup);
  }

  void ensureSubsystemsHaveDefaultCommands() {
    m_drivetrain.setDefaultCommand(fastDrive);
    m_arm.setDefaultCommand(safelyRestTheArm);
    m_lightShow.setDefaultCommand(showBlue);
    m_intake.setDefaultCommand(stopIntake);
  }

}