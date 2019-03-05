/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoCameraTrackWhenCrossXBoundaryNegitive extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutoCameraTrackWhenCrossXBoundaryNegitive(double xBounday) {
    addSequential(new WaitUntilCrossXBoundaryNegativeCommand(xBounday));
    addSequential(new DrivePathCameraTrack());
  }
}
