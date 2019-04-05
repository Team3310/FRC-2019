/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.auto.routes;

import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.team3310.auto.commands.AutoCameraTrackWhenCrossXBoundary;
import frc.team3310.auto.commands.DriveAbsoluteTurnMP;
import frc.team3310.auto.commands.DriveMotionCommand;
import frc.team3310.auto.commands.DrivePathCameraTrackWithVelocity;
import frc.team3310.auto.commands.LazyLoadCommandGroup;
import frc.team3310.auto.commands.WaitUntilCrossXBoundary.MovingXDirection;
import frc.team3310.robot.Constants;
import frc.team3310.robot.commands.EjectHatch;
import frc.team3310.robot.commands.ElevatorAutoZeroSensor;
import frc.team3310.robot.commands.ElevatorSetPositionMM;
import frc.team3310.robot.commands.IntakeHatch;
import frc.team3310.robot.commands.IntakeHatchArms;
import frc.team3310.robot.commands.ResetSensor;
import frc.team3310.robot.paths.TrajectoryGenerator;
import frc.team3310.robot.subsystems.Intake.HatchArmState;
import frc.team3310.utility.MPSoftwarePIDController.MPSoftwareTurnType;

public class AutoStartLevel1SideRocketFrontBackLow extends LazyLoadCommandGroup {

        public AutoStartLevel1SideRocketFrontBackLow() {
                addParallel(new ElevatorAutoZeroSensor());
                addParallel(new ElevatorSetPositionMM(Constants.HATCH_LEVEL_1));
                addParallel(new AutoCameraTrackWhenCrossXBoundary(160, MovingXDirection.Positive, .60, Constants.finishedAtRocketLimeY));
                addSequential(new DriveMotionCommand(registerTrajectory(
                                TrajectoryGenerator.getInstance().getTrajectorySet().level1StartToRocketFront), true));
                addSequential(new EjectHatch());
                addSequential(new WaitCommand("Eject Break", .45));
                addParallel(new IntakeHatch());
                addSequential(new DriveMotionCommand(registerTrajectory(
                                TrajectoryGenerator.getInstance().getTrajectorySet().rocketFrontToTurn1A), false));

                addParallel(new AutoCameraTrackWhenCrossXBoundary(92, MovingXDirection.Negative, 0.5, Constants.finishedAtCargoLimeY), 5); // 100
                addSequential(new DriveMotionCommand(registerTrajectory(
                                TrajectoryGenerator.getInstance().getTrajectorySet().rocketFrontTurn1AToLoading), false));
                addSequential(new IntakeHatchArms(HatchArmState.IN));
                addSequential(new WaitCommand("Grab Break", .5));

                addSequential(new DriveMotionCommand(registerTrajectory(
                                TrajectoryGenerator.getInstance().getTrajectorySet().loadingToRocketBack), false));
                addSequential(new DriveAbsoluteTurnMP(-150, 240, MPSoftwareTurnType.TANK));
                addSequential(new DrivePathCameraTrackWithVelocity(2, Constants.finishedAtCargoLimeY));
                // addSequential(new WaitCommand("Eject Pause", .25));
                // addSequential(new EjectHatch());
                // addParallel(new AutoCameraTrackWhenCrossXBoundary(290, MovingXDirection.Negative, 0.5, Constants.finishedAtRocketLimeY)); // 305
                // addSequential(new DriveMotionCommand(registerTrajectory(
                //                 TrajectoryGenerator.getInstance().getTrajectorySet().turn3ToRocketBack), false));
                //  addSequential(new WaitCommand("Eject Pause", .25));
                //  addSequential(new EjectHatch());
        }
}
