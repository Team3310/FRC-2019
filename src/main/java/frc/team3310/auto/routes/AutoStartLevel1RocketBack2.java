/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.auto.routes;

import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.team3310.auto.commands.DriveAbsoluteTurnMP;
import frc.team3310.auto.commands.DriveMotionCommand;
import frc.team3310.auto.commands.DrivePathCameraTrackWithVelocity;
import frc.team3310.auto.commands.DriveVelocityWithDistance;
import frc.team3310.auto.commands.LazyLoadCommandGroup;
import frc.team3310.robot.Constants;
import frc.team3310.robot.commands.EjectHatch;
import frc.team3310.robot.commands.ElevatorSetPositionMM;
import frc.team3310.robot.commands.IntakeHatch;
import frc.team3310.robot.paths.TrajectoryGenerator;
import frc.team3310.utility.MPSoftwarePIDController.MPSoftwareTurnType;

public class AutoStartLevel1RocketBack2 extends LazyLoadCommandGroup {
  /**
   * Add your docs here.
   */
  public AutoStartLevel1RocketBack2() {
    addSequential(new DriveMotionCommand(registerTrajectory(
        TrajectoryGenerator.getInstance().getTrajectorySet().level1StartReversedToRocketBack), true));
    addParallel(new ElevatorSetPositionMM(Constants.AUTO_HATCH_LEVEL_1));
    addSequential(new DriveAbsoluteTurnMP(45, 240, MPSoftwareTurnType.TANK));
    addSequential(new DrivePathCameraTrackWithVelocity(2, Constants.finishedAtCargoLimeY));
    addSequential(new WaitCommand("Eject Pause", .25));
    addSequential(new EjectHatch());
    addSequential(new DriveVelocityWithDistance(-60, -18));
    addParallel(new IntakeHatch());
    addSequential(new DriveAbsoluteTurnMP(-45, 240, MPSoftwareTurnType.TANK));
  }
}
