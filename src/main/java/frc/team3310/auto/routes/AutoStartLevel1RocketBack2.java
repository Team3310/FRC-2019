/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.auto.routes;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.team3310.auto.commands.AutoCameraTrackWhenCrossXBoundary;
import frc.team3310.auto.commands.DriveMotionCommand;
import frc.team3310.auto.commands.WaitUntilCrossXBoundary.MovingXDirection;
import frc.team3310.robot.Constants;
import frc.team3310.robot.commands.EjectHatch;
import frc.team3310.robot.paths.TrajectoryGenerator;

public class AutoStartLevel1RocketBack2 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutoStartLevel1RocketBack2() {
    addSequential(
        new DriveMotionCommand(TrajectoryGenerator.getInstance().getTrajectorySet().level1StartToRocketBack, false));
    addParallel(
        new AutoCameraTrackWhenCrossXBoundary(305, MovingXDirection.Negative, 0.6, Constants.finishedAtRocketLimeY));
    addSequential(
        new DriveMotionCommand(TrajectoryGenerator.getInstance().getTrajectorySet().turn3ToRocketBack, false));
    addSequential(new WaitCommand("Eject Pause", .25));
    addSequential(new EjectHatch());
  }
}
