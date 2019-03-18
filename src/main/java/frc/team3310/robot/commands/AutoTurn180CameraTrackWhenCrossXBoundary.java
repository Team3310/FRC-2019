/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team3310.robot.commands.WaitUntilCrossXBoundary.MovingXDirection;
import frc.team3310.utility.MPSoftwarePIDController.MPSoftwareTurnType;

public class AutoTurn180CameraTrackWhenCrossXBoundary extends CommandGroup {

  public AutoTurn180CameraTrackWhenCrossXBoundary(double xBoundary, MovingXDirection movingDirection) {
    this(xBoundary, movingDirection, 1.0);
  }

  public AutoTurn180CameraTrackWhenCrossXBoundary(double xBoundary, MovingXDirection movingDirection,
      double velocityScale) {
    addSequential(new WaitUntilCrossXBoundary(xBoundary, movingDirection));
    addSequential(new DriveAbsoluteTurnMP(180, 150, MPSoftwareTurnType.TANK));
    // addSequential(new DrivePathCameraTrack(velocityScale));
  }
}
