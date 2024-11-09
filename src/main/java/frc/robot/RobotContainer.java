// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;
// import org.photonvision.PhotonCamera;

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

    // Whether the driver wants to use the Logitech X3D flightstick controller.
    // If false, assume driver is using an Xbox controller.
    public static final boolean kUseFlightstick = false;
  }

  // SmartDashboard interface.
  SendableChooser<Command> autoChooser;

  // Sensors
  // final PhotonCamera frontCamera = new PhotonCamera("ChurroVision");

  // Logitech flight controller button and joystick axes assignments.
  // Driver uses this controller.
  final LogitechX3D x3dController = new LogitechX3D(Constants.kFlightstickControllerPort);
  final Trigger button1Trigger = new JoystickButton(x3dController, 1); // trigger
  final Trigger button2Trigger = new JoystickButton(x3dController, 2); // side thumb button
  final Trigger button3Trigger = new JoystickButton(x3dController, 3); // hook up
  final Trigger button5Trigger = new JoystickButton(x3dController, 5); // hook down
  final Trigger button7Trigger = new JoystickButton(x3dController, 7); // numbered buttons...
  final Trigger button8Trigger = new JoystickButton(x3dController, 8);
  final Trigger button9Trigger = new JoystickButton(x3dController, 9);
  final Trigger button10Trigger = new JoystickButton(x3dController, 10);
  final Trigger button11Trigger = new JoystickButton(x3dController, 11);
  final Trigger button12Trigger = new JoystickButton(x3dController, 12);

  final DoubleSupplier forwardAxisFlightstick = x3dController::getY;
  final DoubleSupplier sidewaysAxisFlightstick = x3dController::getX;
  final DoubleSupplier rotationAxisFlightstick = x3dController::getTwist;
  final DoubleSupplier sliderAxisFlightstick = x3dController::getThrottle;

  // // Driver controller.
  final XboxController driverController = new XboxController(Constants.kDriverControllerPort);
  final Trigger startButtonDriver = new JoystickButton(driverController, Button.kStart.value);
  final Trigger backButtonDriver = new JoystickButton(driverController, Button.kStart.value);
  final Trigger leftBumperDriver = new JoystickButton(driverController, Button.kLeftBumper.value);
  final Trigger rightBumperDriver = new JoystickButton(driverController, Button.kRightBumper.value);
  final Trigger startAndBackButtonDriver = new Trigger(() -> {
    return startButtonDriver.getAsBoolean() && backButtonDriver.getAsBoolean();
  });
  final Trigger aButtonDriver = new JoystickButton(driverController, Button.kA.value);
  final Trigger bButtonDriver = new JoystickButton(driverController, Button.kB.value);
  final Trigger yButtonDriver = new JoystickButton(driverController, Button.kY.value);
  final Trigger xButtonDriver = new JoystickButton(driverController, Button.kX.value);
  final Trigger rightjoyAnalogTrigger = new Trigger(() -> {
    boolean triggerIsPressedEnough = driverController.getRightTriggerAxis() > 0.28;
    return triggerIsPressedEnough;
  });
  final Trigger leftJoyAnalogTrigger = new Trigger(() -> {
    boolean triggerIsPressedEnough = driverController.getLeftTriggerAxis() > 0.28;
    return triggerIsPressedEnough;
  });

  final DoubleSupplier forwardAxisXbox = driverController::getLeftY;
  final DoubleSupplier sidewaysAxisXbox = driverController::getLeftX;
  final DoubleSupplier rotationAxisXbox = driverController::getRightX;

  // Operator controller.
  final XboxController operatorController = new XboxController(Constants.kOperatorrControllerPort);
  final Trigger leftBumperOperator = new JoystickButton(operatorController, Button.kLeftBumper.value);
  // final Trigger rightBumperOperator = new JoystickButton(operatorController,
  // Button.kRightBumper.value);
  final Trigger aButtonOperator = new JoystickButton(operatorController, Button.kA.value);
  final Trigger xButtonOperator = new JoystickButton(operatorController, Button.kX.value);
  final Trigger bButtonOperator = new JoystickButton(operatorController, Button.kB.value);
  final Trigger yButtonOperator = new JoystickButton(operatorController, Button.kY.value);
  final Trigger startButtonOperator = new JoystickButton(operatorController, Button.kStart.value);
  final Trigger backButtonOperator = new JoystickButton(operatorController, Button.kBack.value);
  final Trigger povUpOperator = new POVButton(operatorController, 0);
  final Trigger povDownOperator = new POVButton(operatorController, 180);
  final Trigger leftjoyTrigger = new JoystickButton(operatorController, Button.kLeftStick.value);
  final Trigger rightjoyTrigger = new JoystickButton(operatorController,
      Button.kRightStick.value);

  // Variables that will be set to either xbox or flightstick axes in
  // configureButtonBindings().
  DoubleSupplier driverForwardAxis;
  DoubleSupplier driverSidewaysAxis;
  DoubleSupplier driverRotationAxis;

  // JJ's comments on what he wants each button to do
  // 1 shoot
  // 2 slow mode

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
          -MathUtil.applyDeadband(driverForwardAxis.getAsDouble() * Constants.kSlowDriveScaling,
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(driverSidewaysAxis.getAsDouble() * Constants.kSlowDriveScaling,
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(driverRotationAxis.getAsDouble() * Constants.kSlowDriveScaling,
              Constants.kDriveDeadband),
          true, true),
      drivetrain);

  final Command superSlowDrive = new RunCommand(
      () -> drivetrain.drive(
          -MathUtil.applyDeadband(driverForwardAxis.getAsDouble() * Constants.kSuperSlowDriveScaling,
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(driverSidewaysAxis.getAsDouble() * Constants.kSuperSlowDriveScaling,
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(rotationAxisFlightstick.getAsDouble() * Constants.kSuperSlowDriveScaling,
              Constants.kDriveDeadband),
          true, true),
      drivetrain);

  final Command fastDrive = new RunCommand(
      () -> drivetrain.drive(
          -MathUtil.applyDeadband(driverForwardAxis.getAsDouble(),
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(driverSidewaysAxis.getAsDouble(),
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(driverRotationAxis.getAsDouble(),
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

    if (Constants.kUseFlightstick) {
      driverForwardAxis = forwardAxisFlightstick;
      driverSidewaysAxis = sidewaysAxisFlightstick;
      driverRotationAxis = rotationAxisFlightstick;

      // Driver Joystick
      button1Trigger.whileTrue(shootDefault);
      button2Trigger.whileTrue(slowDrive);
      button3Trigger.whileTrue(goUpWNoSafety);
      button5Trigger.whileTrue(goDownWNoSafety);
      button7Trigger.whileTrue(recalibrateDrivetrain);
      button8Trigger.whileTrue(prepAmp);
      button9Trigger.whileTrue(eject);
      button10Trigger.whileTrue(anchorInPlace);
      button11Trigger.whileTrue(betterIntakeWeMadeInWorlds);
      button12Trigger.whileTrue(prepShot);
    } else {
      driverForwardAxis = forwardAxisXbox;
      driverSidewaysAxis = sidewaysAxisXbox;
      driverRotationAxis = rotationAxisXbox;

      // Driver Controller
      xButtonDriver.whileTrue(shootSpeaker);
      yButtonDriver.whileTrue(prepAmp);
      aButtonDriver.whileTrue(goUpWNoSafety);
      bButtonDriver.whileTrue(goDownWNoSafety);
      rightBumperDriver.whileTrue(shootDefault);
      leftBumperDriver.whileTrue(slowDrive);
      rightjoyAnalogTrigger.whileTrue(prepShot);
      leftJoyAnalogTrigger.whileTrue(betterIntakeWeMadeInWorlds);
      backButtonDriver.whileTrue(recalibrateDrivetrain);
      startButtonDriver.whileTrue(moveMid);
    }

    // Operator
    aButtonOperator.whileTrue(betterIntakeWeMadeInWorlds);
    yButtonOperator.whileTrue(prepAmp);
    bButtonOperator.whileTrue(moveMid);
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