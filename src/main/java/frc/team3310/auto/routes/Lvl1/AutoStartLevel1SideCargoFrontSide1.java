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
import frc.team3310.auto.commands.LazyLoadCommandGroup;
import frc.team3310.auto.commands.WaitUntilCrossXBoundary.MovingXDirection;
import frc.team3310.robot.Constants;
import frc.team3310.robot.commands.EjectHatch;
import frc.team3310.robot.commands.ElevatorSetPositionMM;
import frc.team3310.robot.commands.IntakeHatch;
import frc.team3310.robot.commands.IntakeHatchArms;
import frc.team3310.robot.paths.TrajectoryGenerator;
import frc.team3310.robot.subsystems.Intake.HatchArmState;
import frc.team3310.utility.MPSoftwarePIDController.MPSoftwareTurnType;

public class AutoStartLevel1SideCargoFrontSide1 extends LazyLoadCommandGroup {

        public AutoStartLevel1SideCargoFrontSide1() {
                addParallel(new ElevatorSetPositionMM(Constants.AUTO_CARGO_LEVEL_1));

                addParallel(new AutoCameraTrackWhenCrossXBoundary(175, MovingXDirection.Positive, 0.6,
                                Constants.finishedAtCargoLimeY, Constants.finishedAtCargoUlt));
                addSequential(new DriveMotionCommand(registerTrajectory(
                                TrajectoryGenerator.getInstance().getTrajectorySet().level1StartToCargoFront), true));
                addSequential(new EjectHatch());
                addSequential(new WaitCommand("Eject Break", .25));
                addParallel(new IntakeHatch());
                addSequential(new DriveMotionCommand(registerTrajectory(
                                TrajectoryGenerator.getInstance().getTrajectorySet().cargoFrontToTurn1), false));

                addParallel(new AutoCameraTrackWhenCrossXBoundary(85, MovingXDirection.Negative, 0.1,
                                Constants.finshedAtLoadingLimeY, Constants.finishedAtLoadingUlt));
                addSequential(new DriveMotionCommand(registerTrajectory(
                                TrajectoryGenerator.getInstance().getTrajectorySet().cargoFrontTurn1ToLoading), false),
                                4);
                addSequential(new IntakeHatchArms(HatchArmState.IN));
                addSequential(new WaitCommand("Grab Break", .35));
                addSequential(new DriveMotionCommand(registerTrajectory(
                        TrajectoryGenerator.getInstance().getTrajectorySet().loadingToCargoSide), false));
                addSequential(new DriveAbsoluteTurnMP(90, 240, MPSoftwareTurnType.TANK));
                addSequential(new DrivePathCameraTrackWithVelocity(2, Constants.finishedAtCargoLimeY,Constants.finishedAtCargoUlt));
        }
}
