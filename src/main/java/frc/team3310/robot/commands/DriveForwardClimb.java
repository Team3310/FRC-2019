package frc.team3310.robot.commands;

import frc.team3310.robot.Robot;
import frc.team3310.robot.subsystems.Drive.DriveControlMode;

/**
 *
 */
public class DriveForwardClimb extends ExtraTimeoutCommand {

  private double speed;
  private boolean isRear;
  private double EXTRA_STOP_TIME = 0.8;

  public DriveForwardClimb(double speed, boolean isRear) {
    this.speed = speed;
    this.isRear = isRear;
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    setTimeout(3);
    resetExtraOneTimer();
    Robot.drive.driveForwardClimb(speed);
    System.out.println("Start drive");
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
    if ((isRear && Robot.elevator.getPlatformDetectRear()) || (!isRear && Robot.elevator.getPlatformDetectFront())) {
      Robot.drive.setControlMode(DriveControlMode.JOYSTICK);
      Robot.drive.driveForwardClimb(-1);
      startExtraOneTimeout(EXTRA_STOP_TIME);
      System.out.println("Found sensor edge");
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    return isExtraOneTimedOut() || isTimedOut();
  }

  // Called once after isFinished returns true
  protected void end() {
    Robot.drive.driveForwardClimb(0);
    Robot.drive.setControlMode(DriveControlMode.JOYSTICK);
    System.out.println("Finished Edge");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    Robot.drive.driveForwardClimb(0);
    Robot.drive.setControlMode(DriveControlMode.JOYSTICK);

  }
}
