/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.auto.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.Robot;
import frc.team3310.robot.paths.TrajectoryGenerator.RightLeftAutonSide;
import frc.team3310.robot.subsystems.Drive;
import frc.team3310.utility.lib.control.RobotStatus;
import frc.team3310.utility.lib.geometry.Pose2dWithCurvature;
import frc.team3310.utility.lib.trajectory.LazyLoadTrajectory;
import frc.team3310.utility.lib.trajectory.MirroredTrajectory;
import frc.team3310.utility.lib.trajectory.TimedView;
import frc.team3310.utility.lib.trajectory.TrajectoryIterator;
import frc.team3310.utility.lib.trajectory.timing.TimedState;

public class DriveMotionCommand extends Command {

  private final LazyLoadTrajectory mLazyLoadTrajectory;
  private final boolean mResetPose;

  public DriveMotionCommand(LazyLoadTrajectory lazyLoadTrajectory, boolean resetPose) {
    mLazyLoadTrajectory = lazyLoadTrajectory;
    mResetPose = resetPose;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    MirroredTrajectory mirroredTrajectory = mLazyLoadTrajectory.getTrajectory();
    TrajectoryIterator<TimedState<Pose2dWithCurvature>> mTrajectory = null;
    if (Robot.trajectoryGenerator.getRightLeftAutonSide() == RightLeftAutonSide.RIGHT) {
      mTrajectory = new TrajectoryIterator<TimedState<Pose2dWithCurvature>>(new TimedView<>(mirroredTrajectory.right));
    }  
    else {
      mTrajectory = new TrajectoryIterator<TimedState<Pose2dWithCurvature>>(new TimedView<>(mirroredTrajectory.left));
    }
    System.out.println("Starting trajectory on " + Robot.trajectoryGenerator.getRightLeftAutonSide() + " side! (length=" + mTrajectory.getRemainingProgress() + ")");
    if (mResetPose) {
      RobotStatus.getInstance().reset(Timer.getFPGATimestamp(), mTrajectory.getState().state().getPose());
    }
//    Robot.drive.startLogging();
    Drive.getInstance().setTrajectory(mTrajectory);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (Drive.getInstance().isDoneWithTrajectory()) {
      System.out.println("Trajectory finished");
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
//    Robot.drive.stopLogging();
    System.out.println("Time to eject Path done ");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
