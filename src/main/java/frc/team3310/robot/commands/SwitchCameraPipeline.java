/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.Robot;

public class SwitchCameraPipeline extends Command {
  private int cameraPipeline;

  public SwitchCameraPipeline(int cameraPipeline) {
    this.cameraPipeline = cameraPipeline;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drive.setPipeline(cameraPipeline);
//    System.out.println("Switch Camera Pipeline = " + cameraPipeline);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {

  }
}
