// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.helpers.LogitechX3D;
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
    public static final int kFlightstickControllerPort = 2;
    public static final double kDriveDeadband = 0.1;
    public static final double kSlowDriveScaling = 0.25;
    public static final double kSuperSlowDriveScaling = .15;
  }

  // SmartDashboard interface.
  SendableChooser<Command> autoChooser;

  // // Driver controller.
  final XboxController driverXboxController = new XboxController(Constants.kDriverControllerPort);
  final Trigger startButtonDriver = new JoystickButton(driverXboxController, Button.kStart.value);
  final Trigger backButtonDriver = new JoystickButton(driverXboxController, Button.kStart.value);
  final Trigger leftBumperDriver = new JoystickButton(driverXboxController, Button.kLeftBumper.value);
  final Trigger rightBumperDriver = new JoystickButton(driverXboxController, Button.kRightBumper.value);

  final Trigger startAndBackButtonDriver = new Trigger(() -> {
    return startButtonDriver.getAsBoolean() && backButtonDriver.getAsBoolean();
  });

  final LogitechX3D x3dController = new LogitechX3D(Constants.kFlightstickControllerPort);
  final Trigger BackButtonDriver = new JoystickButton(x3dController, 7);
  final Trigger StartButtonDriver = new JoystickButton(x3dController, 8);
  final Trigger LeftButtonDriver = new JoystickButton(x3dController, 9);
  final Trigger RightButtonDriver = new JoystickButton(x3dController, 9);

  final DoubleSupplier forwardAxis = x3dController::getY;
  final DoubleSupplier sidewaysAxis = x3dController::getX;
  final DoubleSupplier rotationAxis = x3dController::getTwist;
  // final DoubleSupplier forwardAxis = driverXboxController::getLeftY;
  // final DoubleSupplier sidewaysAxis = driverXboxController::getLeftX;
  // final DoubleSupplier rotationAxis = driverXboxController::getRightX;

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
  final Trigger rightjoyAnalogTrigger = new Trigger(() -> {
    boolean triggerIsPressedEnough = operatorController.getRightTriggerAxis() > 0.28;
    return triggerIsPressedEnough;
  });

  // All of the subsystems.
  final Drivetrain drivetrain = new Drivetrain();
  final Arm arm = new Arm();
  final Intake intake = new Intake();
  final LightShow lightShow = new LightShow();
  final Shooter shooter = new Shooter();
  final Climber climber = new Climber();

  // States of the robot.
  final Trigger armIsHigh = new Trigger(arm::armIsHigh);
  final Trigger flywheelsAreReady = new Trigger(shooter::flywheelsAreAtTarget);
  // final Trigger noteDetected = ???;

  // All of the commands the robot can do.
  final Command prepShot = new RunCommand(shooter::runFlywheelForSpeaker, shooter)
      .alongWith(new RunCommand(arm::move_speaker, arm));

  final Command prepAmp = new RunCommand(shooter::runAmpYeeter, shooter)
      .alongWith(new RunCommand(arm::move_amp, arm));

  final Command shootDefault = new RunCommand(intake::yoinkTheRings, intake);

  // final Command yoinkNote = new RunCommand(shooter::runFlyWheel,
  // shooter).until(shooter::isFlyWheelReady)
  // .andThen(new RunCommand(intake::yoinkTheRings, intake).withTimeout(3));
  final double pullAwayFromShooterTimeout = 0.10;
  final Command stopFlyWheel = new RunCommand(shooter::stopFlyWheel, shooter);
  final Command runIntake = new RunCommand(intake::yoinkTheRings, intake)
      .alongWith(new RunCommand(shooter::reverseAmpYeeter, shooter));
  final Command prepIntake = new RunCommand(shooter::reverseAmpYeeter, shooter).withTimeout(.1);
  final Command stopIntake = new RunCommand(intake::deuceTheRings, intake).withTimeout(pullAwayFromShooterTimeout)
      .andThen(new RunCommand(intake::stopThePlan, intake));
  final Command betterIntakeWeMadeInWorlds = (prepIntake).andThen(runIntake);

  final Command showDefaultColor = new RunCommand(lightShow::disable, lightShow);
  final Command showGreen = new RunCommand(lightShow::greengohappy, lightShow);
  final Command showRed = new RunCommand(lightShow::redgoboom, lightShow);

  final Command anchorInPlace = new RunCommand(() -> drivetrain.setXFormation(), drivetrain);
  final Command recalibrateDrivetrain = new RunCommand(() -> drivetrain.recalibrateDrivetrain(), drivetrain);

  final Command moveArmForSpeaker = new RunCommand(arm::move_speaker, arm);
  final Command moveArmForAmp = new RunCommand(arm::move_amp, arm);
  final Command moveArmForDefault = new RunCommand(arm::move_Default, arm);
  final Command moveMid = new RunCommand(arm::move_mid, arm)
      .alongWith(new RunCommand(shooter::runFlywheelForSpeaker, shooter));

  final Command goUp = new RunCommand(climber::goUp, climber);
  final Command goDown = new RunCommand(climber::goDown, climber);
  final Command stay = new RunCommand(climber::stay, climber);
  final Command goDownWNoSafety = new RunCommand(climber::goDownWNoSafety, climber);
  final Command goUpWNoSafety = new RunCommand(climber::goUpWNoSafety, climber);
  final Command eject = new RunCommand(arm::move_Eject, arm).withTimeout(1)
      .andThen(new RunCommand(intake::ejectNow, intake))
      .alongWith(new RunCommand(shooter::reverseAmpYeeter, shooter));

  final Command slowDrive = new RunCommand(
      () -> drivetrain.drive(
          -MathUtil.applyDeadband(forwardAxis.getAsDouble() * Constants.kSlowDriveScaling,
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(sidewaysAxis.getAsDouble() * Constants.kSlowDriveScaling,
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(rotationAxis.getAsDouble() * Constants.kSlowDriveScaling,
              Constants.kDriveDeadband),
          true, true),
      drivetrain);

  final Command superSlowDrive = new RunCommand(
      () -> drivetrain.drive(
          -MathUtil.applyDeadband(forwardAxis.getAsDouble() * Constants.kSuperSlowDriveScaling,
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(sidewaysAxis.getAsDouble() * Constants.kSuperSlowDriveScaling,
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(rotationAxis.getAsDouble() * Constants.kSuperSlowDriveScaling,
              Constants.kDriveDeadband),
          true, true),
      drivetrain);

  final Command fastDrive = new RunCommand(
      () -> drivetrain.drive(
          -MathUtil.applyDeadband(forwardAxis.getAsDouble(),
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(sidewaysAxis.getAsDouble(),
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(rotationAxis.getAsDouble(),
              Constants.kDriveDeadband),
          true, true),
      drivetrain);

  // For autonomous mode.
  final Command autoPullNoteAwayFromShooter = new RunCommand(intake::deuceTheRings, intake)
      .withTimeout(pullAwayFromShooterTimeout)
      .andThen(new InstantCommand(intake::stopThePlan, intake));
  final Command autoPrepareFlywheelForOldShot = new RunCommand(shooter::runFlywheelForSpeaker, shooter)
      .withTimeout(0.8);
  final Command autoFeedIntoFlywheelForOldShot = new RunCommand(shooter::runFlywheelForSpeaker, shooter)
      .alongWith(new RunCommand(intake::yoinkTheRings, intake)).withTimeout(.25);
  final Command autoAim = new RunCommand(arm::move_speaker, arm).withTimeout(.5);
  final Command shootSpeaker = autoPullNoteAwayFromShooter.alongWith(autoAim)
      .andThen(autoPrepareFlywheelForOldShot)
      .andThen(autoFeedIntoFlywheelForOldShot);

  final Command intakeForThreeSeconds = new RunCommand(arm::move_Default, arm)
      .alongWith(new InstantCommand(intake::yoinkTheRings, intake))
      .alongWith(new RunCommand(shooter::reverseAmpYeeter, shooter)).finallyDo(intake::stopThePlan);

  final Command waitForTeammates = new WaitCommand(5);

  final Command chaos = new RunCommand(arm::move_Default, arm)
      .alongWith(new RunCommand(intake::ejectNow, intake));
  final Command volcanoChaos = new RunCommand(arm::move_Default, arm)
      .alongWith(new RunCommand(intake::isYoinking, intake))
      .alongWith(new RunCommand(shooter::runButtDustYeet, shooter));

  final Command autoFeedIntoFlyWheels = new RunCommand(shooter::runFlywheelForSpeaker, shooter)
      .alongWith(new RunCommand(intake::yoinkTheRings, intake)).withTimeout(.25);
  final Command autoPrepFlyWheels = (new RunCommand(intake::deuceTheRings, intake)
      .withTimeout(pullAwayFromShooterTimeout)
      .andThen(new InstantCommand(intake::stopThePlan, intake)))
      .andThen(new RunCommand(shooter::runFlywheelForSpeaker, shooter)
          .alongWith(new RunCommand(arm::move_speaker, arm)));

  final Command autoPartPullNoteAwayFromShooterMid = new RunCommand(intake::deuceTheRings, intake)
      .withTimeout(pullAwayFromShooterTimeout)
      .andThen(new InstantCommand(intake::stopThePlan, intake));
  final Command autoPartPrepareFlywheelForMidShot = new RunCommand(shooter::runFlywheelForSpeaker, shooter)
      .withTimeout(0.8);
  final Command autoPartFeedIntoFlywheelForMidShot = new RunCommand(shooter::runFlywheelForSpeaker, shooter)
      .alongWith(new RunCommand(intake::yoinkTheRings, intake)).withTimeout(.25);
  final Command autoPartAimMid = new RunCommand(arm::move_mid, arm).withTimeout(.5);
  final Command autoMidSpeaker = autoPartPullNoteAwayFromShooterMid.alongWith(autoPartAimMid)
      .andThen(autoPartPrepareFlywheelForMidShot)
      .andThen(autoPartFeedIntoFlywheelForMidShot);

  final Command autoFeedIntoFlyWheelsMid = new RunCommand(shooter::runFlywheelForSpeaker, shooter)
      .alongWith(new RunCommand(intake::yoinkTheRings, intake)).withTimeout(.25);
  final Command autoPrepFlyWheelsMid = (new RunCommand(intake::deuceTheRings,
      intake)
      .withTimeout(pullAwayFromShooterTimeout)
      .andThen(new InstantCommand(intake::stopThePlan, intake)))
      .andThen(new RunCommand(shooter::runFlywheelForSpeaker, shooter)
          .alongWith(new RunCommand(arm::move_mid, arm)));

  public RobotContainer() {
    configureButtonBindings();
    ensureSubsystemsHaveDefaultCommands();
    createAutonomousSelector();
    lightShow.disable();
  }

  /**
   * This is called by the system when automomous runs, and it
   * should return the command you want to execute when automous
   * mode begins.
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
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
    NamedCommands.registerCommand("chaos", chaos);
    NamedCommands.registerCommand("autoFeedIntoFlyWheels", autoFeedIntoFlyWheels);
    NamedCommands.registerCommand("autoPrepFlyWheels", autoPrepFlyWheels);
    NamedCommands.registerCommand("volcanoChaos", volcanoChaos);
    NamedCommands.registerCommand("midSpeaker", autoMidSpeaker);
    NamedCommands.registerCommand("autoPrepareFlywheelForMidShot", autoPrepFlyWheelsMid);
    NamedCommands.registerCommand("autoFeedIntoFlywheelForMidShot", autoFeedIntoFlyWheelsMid);
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
  }

  void configureButtonBindings() {

    // Driver
    leftBumperDriver.whileTrue(anchorInPlace);
    rightBumperDriver.whileTrue(slowDrive);
    startAndBackButtonDriver.whileTrue(recalibrateDrivetrain);

    // Operator
    aButtonOperator.whileTrue(betterIntakeWeMadeInWorlds);
    yButtonOperator.whileTrue(prepAmp);
    xButtonOperator.whileTrue(prepShot);
    bButtonOperator.whileTrue(moveMid);
    rightBumperOperator.whileTrue(shootDefault);
    rightjoyAnalogTrigger.whileTrue(shootDefault);

    startButtonOperator.whileTrue(goUpWNoSafety);
    backButtonOperator.whileTrue(goDownWNoSafety);
    povUpOperator.whileTrue(goUp);
    povDownOperator.whileTrue(goDown);

    rightjoyTrigger.whileTrue(eject);

    // Sensors
    armIsHigh.whileTrue(superSlowDrive);
    flywheelsAreReady.whileTrue(showGreen);
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
// Fix setDefaultCommand fpr drivetrain after presenting to kids