package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.Robot;
import frc.team3310.robot.subsystems.Drive.DriveControlMode;

/**
 *
 */
public class DriveHoldClimbWheel extends Command {

  private double distance;

  public DriveHoldClimbWheel(double distance) {
    this.distance = distance;
    // requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    Robot.drive.resetMiddleEncoder();
    Robot.drive.setMiddleDriveMotionMagicPosition(distance);

    // System.out.println("Start drive");
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
  }


  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    return Robot.drive.hasFinishedTrajectory();
  }

  // Called once after isFinished returns true
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    end();
  }
}
