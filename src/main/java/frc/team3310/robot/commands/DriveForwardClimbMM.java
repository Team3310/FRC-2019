package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.Robot;
import frc.team3310.robot.subsystems.Drive.DriveControlMode;

/**
 *
 */
public class DriveForwardClimbMM extends Command {

  private double distance;
  private boolean isRear;
  private boolean edgeDetected;

  public DriveForwardClimbMM(double distance, boolean isRear) {
    this.distance = distance;
    this.isRear = isRear;
    // requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    edgeDetected = false;
    Robot.drive.resetMiddleEncoder();
    Robot.drive.setMiddleDriveMotionMagicPosition(distance);
    Robot.drive.setVelocitySetpoint(20, 20);

    // System.out.println("Start drive");
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
    if ((isRear && Robot.elevator.getPlatformDetectRear()) || (!isRear && Robot.elevator.getPlatformDetectFront())) {
      // Robot.drive.setControlMode(DriveControlMode.JOYSTICK);
      Robot.drive.setControlMode(DriveControlMode.HOLD);
      double currentDistance = Robot.drive.getMiddleEncoderInches();
      Robot.drive.setMiddleDriveMotionMagicPosition(currentDistance);
      edgeDetected = true;
      // System.out.println("Found sensor edge");
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    return Robot.drive.hasFinishedTrajectory() && edgeDetected == true;
//    return Robot.drive.hasFinishedTrajectory() || edgeDetected == true;
 }

  // Called once after isFinished returns true
  protected void end() {
    Robot.drive.setControlMode(DriveControlMode.JOYSTICK);
    // Robot.drive.driveForwardClimb(0);
    // System.out.println("Finished Drive Forward = 0");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    end();
  }
}
