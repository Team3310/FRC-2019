/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team3310.robot.commands.WaitUntilCrossYBoundary.MovingYDirection;

public class AutoCameraTrackWhenCrossYBoundary extends CommandGroup {
 
  public AutoCameraTrackWhenCrossYBoundary(double yBoundaryForRightSideAuton, MovingYDirection movingDirection) {
    this(yBoundaryForRightSideAuton, movingDirection, 1.0);
  }

  public AutoCameraTrackWhenCrossYBoundary(double yBoundaryForRightSideAuton, MovingYDirection movingDirection, double velocityScale) {
    addSequential(new WaitUntilCrossYBoundary(yBoundaryForRightSideAuton, movingDirection));
    addSequential(new DrivePathCameraTrack(velocityScale));
  }
}
