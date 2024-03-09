// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LightShow;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

  static final class Constants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorrControllerPort = 1;
    public static final double kDriveDeadband = 0.1;
    public static final double kSlowDriveScaling = 0.4;
  }

  // SmartDashboard interface.
  SendableChooser<Command> autoChooser;

  // Driver controller.
  final XboxController driverController = new XboxController(Constants.kDriverControllerPort);
  final Trigger startButtonDriver = new JoystickButton(driverController, Button.kStart.value);
  final Trigger backButtonDriver = new JoystickButton(driverController, Button.kStart.value);
  final Trigger leftBumperDriver = new JoystickButton(driverController, Button.kLeftBumper.value);
  final Trigger rightBumperDriver = new JoystickButton(driverController, Button.kRightBumper.value);
  final Trigger startAndBackButtonDriver = new Trigger(() -> {
    return startButtonDriver.getAsBoolean() && backButtonDriver.getAsBoolean();
  });
  // Operator controller.
  final XboxController operatorController = new XboxController(Constants.kOperatorrControllerPort);
  final Trigger leftBumperOperator = new JoystickButton(operatorController, Button.kLeftBumper.value);
  final Trigger rightBumperOperator = new JoystickButton(operatorController, Button.kRightBumper.value);
  final Trigger aButtonOperator = new JoystickButton(operatorController, Button.kA.value);
  final Trigger xButtonOperator = new JoystickButton(operatorController, Button.kX.value);
  final Trigger yButtonOperator = new JoystickButton(operatorController, Button.kY.value);
  final Trigger bButtonOperator = new JoystickButton(operatorController, Button.kB.value);
  final Trigger startButtonOperator = new JoystickButton(operatorController, Button.kStart.value);
  final Trigger backButtonOperator = new JoystickButton(operatorController, Button.kBack.value);
  final Trigger povUpOperator = new POVButton(operatorController, 0);
  final Trigger povDownOperator = new POVButton(operatorController, 180);
  final Trigger leftjoyTrigger = new JoystickButton(operatorController, Button.kLeftStick.value);
  final Trigger rightjoyTrigger = new JoystickButton(operatorController, Button.kRightStick.value);


  // All of the subsystems.
  final Drivetrain drivetrain = new Drivetrain();
  final Arm arm = new Arm();
  final Intake intake = new Intake();
  final LightShow lightShow = new LightShow();
  final Shooter shooter = new Shooter();
  final Climber climber = new Climber();

  // States of the robot.
  final Trigger armIsHigh = new Trigger(arm::armIsHigh);

  // All of the commands the robot can do.
  final Command runFlywheels = new RunCommand(shooter::stopFlyWheel, shooter).withTimeout(.1)
      .andThen(new RunCommand(shooter::runFlywheelForSpeaker, shooter));

  // TODO: make sure they're right
  final Command shootDefault = new RunCommand(intake::yoinkTheRings, intake);
  // final Command yoinkNote = new RunCommand(shooter::runFlyWheel,
  // shooter).until(shooter::isFlyWheelReady)
  // .andThen(new RunCommand(intake::yoinkTheRings, intake).withTimeout(3));
  final Command stopFlyWheel = new RunCommand(shooter::stopFlyWheel, shooter);
  final Command runIntake = new RunCommand(intake::yoinkTheRings, intake)
      .alongWith(new RunCommand(shooter::reverseAmpYeeter, shooter));
  final Command stopIntake = new RunCommand(intake::deuceTheRings, intake).withTimeout(.1)
      .andThen(new RunCommand(intake::stopThePlan, intake));
  final Command showDefaultColor = new RunCommand(() -> {
    if (DriverStation.isAutonomous()) {
      lightShow.setPurple();
    } else {
      lightShow.setGreen();
    }
  }, lightShow);

  final Command anchorInPlace = new RunCommand(() -> drivetrain.setXFormation(), drivetrain);
  final Command resetGyro = new RunCommand(() -> drivetrain.resetGyro(), drivetrain);

  final Command moveArmForSpeaker = new RunCommand(arm::move_speaker, arm);
  final Command moveArmForAmp = new RunCommand(arm::move_amp, arm);
  final Command moveArmForDefault = new RunCommand(arm::move_Default, arm);
  final Command runAmpYeeter = new RunCommand(shooter::runAmpYeeter, shooter)
      .alongWith(new RunCommand(intake::yoinkTheRings, intake));

  final Command goUp = new RunCommand(climber::goUp, climber);
  final Command goDown = new RunCommand(climber::goDown, climber);
  final Command stay = new RunCommand(climber::stay, climber);
  final Command goDownWNoSafety = new RunCommand(climber::goDownWNoSafety, climber);
  final Command goUpWNoSafety = new RunCommand(climber::goUpWNoSafety, climber);
  final Command eject = new RunCommand(arm::move_Eject, arm)
  .andThen(new RunCommand(intake::ejectNow, intake));
  // Commands.runOnce(() -> setGoal(kArmOffsetRads), this);

  final Command slowDrive = new RunCommand(
      () -> drivetrain.drive(
          -MathUtil.applyDeadband(driverController.getLeftY() * Constants.kSlowDriveScaling,
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(driverController.getLeftX() * Constants.kSlowDriveScaling,
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(driverController.getRightX() * Constants.kSlowDriveScaling,
              Constants.kDriveDeadband),
          true, true),
      drivetrain);

  final Command fastDrive = new RunCommand(
      () -> drivetrain.drive(
          -MathUtil.applyDeadband(driverController.getLeftY(),
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(driverController.getLeftX(),
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(driverController.getRightX(),
              Constants.kDriveDeadband),
          true, true),
      drivetrain);

  // For autonomous mode.
  final Command autoPullNoteAwayFromShooter = new RunCommand(intake::deuceTheRings, intake).withTimeout(.05)
      .andThen(new InstantCommand(intake::stopThePlan, intake));
  final Command autoPrepareFlywheel = new RunCommand(shooter::runFlywheelForSpeaker, shooter).withTimeout(1.1);
  final Command autoFeedIntoFlywheel = new RunCommand(shooter::runFlywheelForSpeaker, shooter)
      .alongWith(new RunCommand(intake::yoinkTheRings, intake)).withTimeout(1);

  final Command shootSpeaker = autoPullNoteAwayFromShooter
      .andThen(autoPrepareFlywheel)
      .andThen(autoFeedIntoFlywheel);
  final Command intakeForThreeSeconds = new InstantCommand(intake::yoinkTheRings, intake)
      .alongWith(new RunCommand(shooter::reverseAmpYeeter, shooter)).finallyDo(intake::stopThePlan);
  final Command waitForTeammates = new WaitCommand(5);

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
    var command = autoChooser.getSelected();
    return command.finallyDo(drivetrain::resetGyro);
  }

  /**
   * Note that old autonomous modes are not automatically cleared out of the
   * deploy directory on the rio. If you delete an autonomous mode, you might
   * want to first clear the directory on the rio via SSH:
   * ```
   * ssh admin@roboRIO-8048-frc.local "rm -rf /home/lvuser/deploy/pathplanner"
   * ```
   * And then re-deploy to have the most current autonomous modes.
   */
  public void createAutonomousSelector() {
    NamedCommands.registerCommand("shootSpeaker", shootSpeaker);
    NamedCommands.registerCommand("intakeForThreeSeconds", intakeForThreeSeconds);
    NamedCommands.registerCommand("waitForTeammates", waitForTeammates);
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
  }

  void configureButtonBindings() {
    leftBumperDriver.whileTrue(anchorInPlace);
    rightBumperDriver.whileTrue(slowDrive);
    aButtonOperator.whileTrue(runIntake);
    yButtonOperator.whileTrue(moveArmForAmp);
    leftBumperOperator.whileTrue(runFlywheels);
    rightBumperOperator.whileTrue(shootDefault);
    startButtonOperator.whileTrue(goUpWNoSafety);
    backButtonOperator.whileTrue(goDownWNoSafety);
    povUpOperator.whileTrue(goUp);
    povDownOperator.whileTrue(goDown);
    leftjoyTrigger.whileTrue(runAmpYeeter);
    rightjoyTrigger.whileTrue(eject);
    armIsHigh.whileTrue(slowDrive);
    startAndBackButtonDriver.whileTrue(resetGyro);
  }

  void ensureSubsystemsHaveDefaultCommands() {
    drivetrain.setDefaultCommand(fastDrive);
    lightShow.setDefaultCommand(showDefaultColor);
    arm.setDefaultCommand(moveArmForDefault);
    shooter.setDefaultCommand(stopFlyWheel);
    intake.setDefaultCommand(stopIntake);
    climber.setDefaultCommand(stay);
  }

}