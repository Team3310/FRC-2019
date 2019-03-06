/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team3310.utility.lib.geometry.Translation2d;

public class AutoCameraTrackWhenInRegion extends CommandGroup {

  public AutoCameraTrackWhenInRegion(Translation2d bottomLeftForRightSideAuton, Translation2d topRightForRightSideAuton) {
    this(bottomLeftForRightSideAuton, topRightForRightSideAuton, 1.0);
  }

  public AutoCameraTrackWhenInRegion(Translation2d bottomLeftForRightSideAuton, Translation2d topRightForRightSideAuton, double velocityScale) {
    addSequential(new WaitUntilInsideRegion(bottomLeftForRightSideAuton, topRightForRightSideAuton));
    addSequential(new DrivePathCameraTrack(velocityScale));
  }
}
