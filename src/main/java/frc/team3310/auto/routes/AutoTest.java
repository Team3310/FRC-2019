/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.auto.routes;

import frc.team3310.auto.commands.AutoCameraTrackWhenCrossXBoundary;
import frc.team3310.auto.commands.DriveMotionCommand;
import frc.team3310.auto.commands.LazyLoadCommandGroup;
import frc.team3310.auto.commands.WaitUntilCrossXBoundary.MovingXDirection;
import frc.team3310.robot.Constants;
import frc.team3310.robot.commands.ResetRobotToLoadingPose;
import frc.team3310.robot.paths.TrajectoryGenerator;

public class AutoTest extends LazyLoadCommandGroup {
 
  public AutoTest() {
    addParallel(new AutoCameraTrackWhenCrossXBoundary(50, MovingXDirection.Positive, .2, Constants.finishedAtCargoLimeY));
    addSequential(new DriveMotionCommand(registerTrajectory(
      TrajectoryGenerator.getInstance().getTrajectorySet().driveStraight), true));  
    addSequential(new ResetRobotToLoadingPose());
    addSequential(new DriveMotionCommand(registerTrajectory(
      TrajectoryGenerator.getInstance().getTrajectorySet().test), true));  
    

  }
    
}
