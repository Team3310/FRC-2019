package frc.team3310.robot.commands;

import frc.team3310.robot.*;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveForwardClimb extends Command {

  private double speed;

  public DriveForwardClimb(double speed) {
    this.speed = speed;
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    Robot.drive.driveForwardClimb(speed);
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
  }
}
