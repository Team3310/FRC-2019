/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.auto.routes;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team3310.auto.commands.DrivePathCameraTrack;
import frc.team3310.auto.commands.DriveVelocityWithDistance;

public class AutoTest extends CommandGroup {
 
  public AutoTest() {
    // addParallel(new AutoCameraTrackWhenCrossXBoundary(6, MovingXDirection.Positive, 0.5, Constants.finishedAtCargoLimeY));
    addSequential(new DriveVelocityWithDistance(96, 12));
    addSequential(new DrivePathCameraTrack());
  } 
}
