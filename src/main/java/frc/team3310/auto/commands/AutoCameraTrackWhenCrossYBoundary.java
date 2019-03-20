/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.auto.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team3310.auto.commands.WaitUntilCrossYBoundary.MovingYDirection;
import frc.team3310.auto.commands.DrivePathCameraTrack;

public class AutoCameraTrackWhenCrossYBoundary extends CommandGroup {

  public AutoCameraTrackWhenCrossYBoundary(double yBoundaryForRightSideAuton, MovingYDirection movingDirection,
      double velocityScale, double finishedAtLimeY) {
    addSequential(new WaitUntilCrossYBoundary(yBoundaryForRightSideAuton, movingDirection));
    addSequential(new DrivePathCameraTrack(velocityScale, finishedAtLimeY));
  }
}
