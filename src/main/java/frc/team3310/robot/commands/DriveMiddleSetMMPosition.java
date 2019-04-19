/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.Robot;
import frc.team3310.robot.subsystems.Drive;

public class DriveMiddleSetMMPosition extends Command {
  private double targetPositionInches;

  public DriveMiddleSetMMPosition(double targetPositionInches) {
    this.targetPositionInches = targetPositionInches;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drive.setMiddleDriveMotionMagicPosition(targetPositionInches);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (Drive.getInstance().hasFinishedTrajectory()) {
      // System.out.println("Trajectory finished");
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  protected void end() {
    // System.out.println("Middle Drive set MM end");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {

  }
}
