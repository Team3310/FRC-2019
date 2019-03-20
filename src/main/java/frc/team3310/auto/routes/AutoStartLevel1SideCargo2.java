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
import frc.team3310.auto.commands.AutoCameraTrackWhenCrossYBoundary;
import frc.team3310.auto.commands.DriveAbsoluteTurnMP;
import frc.team3310.auto.commands.DriveMotionCommand;
import frc.team3310.auto.commands.WaitUntilCrossXBoundary.MovingXDirection;
import frc.team3310.auto.commands.WaitUntilCrossYBoundary.MovingYDirection;
import frc.team3310.robot.Constants;
import frc.team3310.robot.commands.DriveSetSpeed;
import frc.team3310.robot.commands.EjectHatch;
import frc.team3310.robot.commands.ElevatorSetPositionMM;
import frc.team3310.robot.commands.IntakeHatchArms;
import frc.team3310.robot.paths.TrajectoryGenerator;
import frc.team3310.robot.subsystems.Intake.HatchArmState;
import frc.team3310.utility.MPSoftwarePIDController.MPSoftwareTurnType;

public class AutoStartLevel1SideCargo2 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutoStartLevel1SideCargo2() {
    addParallel(new ElevatorSetPositionMM(Constants.HATCH_LEVEL_1));
    addSequential(new DriveMotionCommand(
                TrajectoryGenerator.getInstance().getTrajectorySet().level1StartToCargoSideTurn, true));
        addParallel(new AutoCameraTrackWhenCrossYBoundary(-58, MovingYDirection.OutsideToInside, 1.0 , Constants.finishedAtCargoLimeY));
        addSequential(
                new DriveMotionCommand(TrajectoryGenerator.getInstance().getTrajectorySet().turnSideCargoToSideCargoScore, false));
        addSequential(new WaitCommand("Eject Pause", .25));
        addSequential(new EjectHatch());
        addSequential(new DriveSetSpeed(-1, .2));
        addSequential(new DriveAbsoluteTurnMP(90, 400, MPSoftwareTurnType.TANK));
        addParallel(new AutoCameraTrackWhenCrossXBoundary(65, MovingXDirection.Negative, 0.7, Constants.finishedAtCargoLimeY));
        addSequential(new DriveMotionCommand(
                TrajectoryGenerator.getInstance().getTrajectorySet().cargoSideScoreToLoading, false));
        addSequential(new IntakeHatchArms(HatchArmState.IN));
        addSequential(new WaitCommand("Grab Break", .25));
  }
}
