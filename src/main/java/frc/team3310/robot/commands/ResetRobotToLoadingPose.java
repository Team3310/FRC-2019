/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.Robot;
import frc.team3310.robot.paths.TrajectoryGenerator.RightLeftAutonSide;
import frc.team3310.utility.lib.control.RobotStatus;
import frc.team3310.utility.lib.geometry.Pose2d;
import frc.team3310.utility.lib.geometry.Rotation2d;
import frc.team3310.utility.lib.geometry.Translation2d;

public class ResetRobotToLoadingPose extends Command {
  public ResetRobotToLoadingPose() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    RobotStatus.getInstance().resetDistanceDriven();
    if (Robot.trajectoryGenerator.getRightLeftAutonSide() == RightLeftAutonSide.RIGHT) {
      RobotStatus.getInstance().reset(Timer.getFPGATimestamp(), new Pose2d(new Translation2d(0.00, -135),
      Rotation2d.fromDegrees(180.0)));		
    }
    else{
      RobotStatus.getInstance().reset(Timer.getFPGATimestamp(), new Pose2d(new Translation2d(0.00, 135),
      Rotation2d.fromDegrees(180.0)));		
    }

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
