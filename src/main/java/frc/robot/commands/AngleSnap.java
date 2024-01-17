package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class AngleSnap extends PIDCommand {

  private final static double kDrivingP = 0.012;
  private final static double kDrivingI = 0;
  private final static double kDrivingD = 0;
  private static final double kTurnToleranceDeg = 1;
  private static final double kTurnRateToleranceDegPerS = 5;

  public AngleSnap(double targetAngleDegrees, Drivetrain drive) {
    super(
        new PIDController(kDrivingP, kDrivingI, kDrivingD),
        // Close loop on heading
        drive::getHeading,
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn robot
        output -> drive.drive(0, 0, output, true, true),
        // Require the drive
        drive);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is
    // stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(kTurnToleranceDeg, kTurnRateToleranceDegPerS);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();

  }
}