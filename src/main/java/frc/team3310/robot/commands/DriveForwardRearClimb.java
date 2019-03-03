package frc.team3310.robot.commands;

import frc.team3310.robot.*;
import frc.team3310.robot.subsystems.Drive.DriveControlMode;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveForwardRearClimb extends Command {

  private double speed;

  public DriveForwardRearClimb(double speed) {
    this.speed = speed;
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    Robot.drive.driveForwardClimb(speed);
    if (speed > 0.0) {
      // Robot.drive.setVelocitySetpoint(40, 40);
    } else {
      // Robot.drive.setControlMode(DriveControlMode.JOYSTICK);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    return true;
    // return Robot.elevator.getPlatformDetectRear();
  }

  // Called once after isFinished returns true
  protected void end() {
    // Robot.drive.driveForwardClimb(0);
    // Robot.drive.setControlMode(DriveControlMode.JOYSTICK);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    // Robot.drive.driveForwardClimb(0);
    // Robot.drive.setControlMode(DriveControlMode.JOYSTICK);

  }
}
