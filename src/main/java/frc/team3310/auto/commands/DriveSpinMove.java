/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.auto.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.Robot;
import frc.team3310.robot.subsystems.Drive;

public class DriveSpinMove extends Command {
  private double targetPositionAngle;
  boolean startedSpinMove = false;

  public DriveSpinMove(double targetPositionAngle) {
    this.targetPositionAngle = targetPositionAngle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Set motion magic drive");
//    Robot.drive.setDriveMotionMagic(targetPositionInches, targetPositionAngle);
    Robot.drive.setVelocitySetpoint(-48, -48);
    setTimeout(0.3);
    startedSpinMove = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (startedSpinMove == false && isTimedOut()) {
      Robot.drive.setDriveSpinMove(targetPositionAngle);
      startedSpinMove = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
 //   if (Drive.getInstance().hasFinishedDriveMotionMagic()) {
    if (startedSpinMove && Drive.getInstance().hasFinishedDSpinMove()) {
        System.out.println("Trajectory finished " + Drive.getInstance().getDriveMotionMagicPosition());
      return true;
    }
    // System.out.println("Motion magic driving...");
    return false;
  }

  // Called once after isFinished returns true
  protected void end() {
    System.out.println("Drive set MM end");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    System.out.println("Drive set MM end");

  }
}
