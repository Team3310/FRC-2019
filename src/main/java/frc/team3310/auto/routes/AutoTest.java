/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.auto.routes;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team3310.auto.commands.DrivePathCameraTrackWithVelocity;
import frc.team3310.robot.Constants;

public class AutoTest extends CommandGroup {
 
  public AutoTest() {
    addSequential(new DrivePathCameraTrackWithVelocity(2, Constants.finishedAtCargoLimeY));
  }
}
