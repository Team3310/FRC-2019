/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.auto.routes.Lvl1;

import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.team3310.auto.commands.AutoCameraTrackWhenCrossXBoundary;
import frc.team3310.auto.commands.DriveAbsoluteTurnMP;
import frc.team3310.auto.commands.DriveMotionCommand;
import frc.team3310.auto.commands.DrivePathCameraTrackWithVelocity;
import frc.team3310.auto.commands.DriveVelocityWithDistance;
import frc.team3310.auto.commands.LazyLoadCommandGroup;
import frc.team3310.auto.commands.WaitUntilCrossXBoundary.MovingXDirection;
import frc.team3310.robot.Constants;
import frc.team3310.robot.commands.EjectHatch;
import frc.team3310.robot.commands.ElevatorSetPositionMM;
import frc.team3310.robot.commands.IntakeHatch;
import frc.team3310.robot.commands.IntakeHatchArms;
import frc.team3310.robot.commands.ResetSensor;
import frc.team3310.robot.paths.TrajectoryGenerator;
import frc.team3310.robot.subsystems.Intake.HatchArmState;
import frc.team3310.utility.MPSoftwarePIDController.MPSoftwareTurnType;

public class AutoStartLevel1RocketBack2 extends LazyLoadCommandGroup {
  /**
   * Add your docs here.
   */
  public AutoStartLevel1RocketBack2() {
    addSequential(new ResetSensor());
    addSequential(new DriveMotionCommand(registerTrajectory(
        TrajectoryGenerator.getInstance().getTrajectorySet().level1StartReversedToRocketBack), true));
    addParallel(new ElevatorSetPositionMM(Constants.AUTO_ROCKET_LEVEL_1));
    addSequential(new DriveAbsoluteTurnMP(30, 240, MPSoftwareTurnType.TANK));
    addSequential(new DrivePathCameraTrackWithVelocity(1.5, Constants.finishedAtRocketLimeY, Constants.finishedAtRocketLimeY));
    addSequential(new EjectHatch());
    addSequential(new WaitCommand("Eject Pause", .35));
    addSequential(new DriveVelocityWithDistance(-60, -32));
    addParallel(new IntakeHatch());
    addSequential(new DriveAbsoluteTurnMP(-20, 240, MPSoftwareTurnType.TANK));
    addParallel(
      new AutoCameraTrackWhenCrossXBoundary(100, MovingXDirection.Negative, 0.65
      , Constants.finshedAtLoadingLimeY, Constants.finishedAtLoadingUlt)); // 100
      addSequential(new DriveMotionCommand(registerTrajectory(
        TrajectoryGenerator.getInstance().getTrajectorySet().rocketBackToLoading), false));

    addSequential(new IntakeHatchArms(HatchArmState.IN));
    addSequential(new WaitCommand("Grab Break", .75));
    addSequential(new DriveMotionCommand(registerTrajectory(
      TrajectoryGenerator.getInstance().getTrajectorySet().loadingToRocketBack), false, false));
    addParallel(new ElevatorSetPositionMM(Constants.HATCH_LEVEL_2));
    addSequential(new DriveAbsoluteTurnMP(30, 240, MPSoftwareTurnType.TANK));
    addSequential(new DrivePathCameraTrackWithVelocity(2, Constants.finishedAtRocketLimeY, Constants.finishedAtRocketUlt));
  }
}
