// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LightShow;

public class RobotContainer {

  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Arm m_arm = new Arm();
  private final Intake m_intake = new Intake(m_arm);
  private final LightShow m_lightShow = new LightShow();

  private final SendableChooser<Command> autoChooser;


  private static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorrControllerPort = 1;
    public static final double kDriveDeadband = 0.1;
  }

  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(OIConstants.kOperatorrControllerPort);

  Command slowDrive;
  Command fastDrive;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("sayHello", Commands.none());
    NamedCommands.registerCommand("sayBye", Commands.none());
  
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
    double slowDriveScaling = 0.4;
    slowDrive = new RunCommand(
        () -> m_drivetrain.drive(
            -MathUtil.applyDeadband(m_driverController.getLeftY() * slowDriveScaling,
                OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getLeftX() * slowDriveScaling,
                OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getRightX() * slowDriveScaling,
                OIConstants.kDriveDeadband),
            true, true),
        m_drivetrain);

    fastDrive = new RunCommand(
        () -> m_drivetrain.drive(
            -MathUtil.applyDeadband(m_driverController.getLeftY(),
                OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getLeftX(),
                OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getRightX(),
                OIConstants.kDriveDeadband),
            true, true),
        m_drivetrain);

    configureButtonBindings();
    ensureSubsystemsHaveDefaultCommands();
    createAutonomousSelector();
  }

  public void handleDisable() {
    createAutonomousSelector();
  }

  public void createAutonomousSelector() {
    // FIXME: need to instantiate the autonomous selector
  }

  private void configureButtonBindings() {

    // Teleop commands for the driver and operator.

    Command anchorInPlace = new RunCommand(() -> m_drivetrain.setXFormation(), m_drivetrain);
    Command resetGyro = new RunCommand(() -> m_drivetrain.resetGyro(), m_drivetrain);

    Command yeet = new RunCommand(m_intake::yeetTheCubes, m_intake);
    Command yoink = new RunCommand(m_intake::yoinkTheCubes, m_intake);

    Command moveArmIntoCalibration = new RunCommand(m_arm::moveIntoCalibrationPosition, m_arm);
    Command resetArmCalibration = new RunCommand(m_arm::resetCalibration, m_arm);
    Command moveToReceive = new RunCommand(() -> m_arm.receiveFromSingleSubstation(-m_operatorController.getLeftY()),
        m_arm);
    Command moveToLow = new RunCommand(() -> m_arm.moveToLow(-m_operatorController.getLeftY()), m_arm);
    Command moveToMid = new RunCommand(() -> m_arm.moveToMid(-m_operatorController.getLeftY()), m_arm);
    Command moveToGroundPickup = new RunCommand(m_arm::receiveFromGround, m_arm);

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

  private void ensureSubsystemsHaveDefaultCommands() {

    var safelyRestTheArm = new RunCommand(m_arm::restTheArm, m_arm);
    var showBlue = new RunCommand(m_lightShow::setBlue, m_lightShow);

    // Set defaults for all subsystems
    m_drivetrain.setDefaultCommand(fastDrive);
    m_arm.setDefaultCommand(safelyRestTheArm);
    m_lightShow.setDefaultCommand(showBlue);
  }

  /**
   * This is called by the system when automomous runs, and it
   * should return the command you want to execute when automous
   * mode begins.
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}