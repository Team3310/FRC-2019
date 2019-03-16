/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.team3310.robot.Constants;
import frc.team3310.robot.commands.WaitUntilCrossXBoundary.MovingXDirection;
import frc.team3310.robot.paths.TrajectoryGenerator;
import frc.team3310.robot.subsystems.Intake.HatchArmState;

public class AutoStartLevel1SideRocketFrontBackLow extends CommandGroup {

        public AutoStartLevel1SideRocketFrontBackLow() {
                addParallel(new ElevatorSetPositionMM(Constants.HATCH_LEVEL_1));

                addParallel(new AutoTurn180CameraTrackWhenCrossXBoundary(160, MovingXDirection.Positive, .60));
                addSequential(new DriveMotionCommand(
                                TrajectoryGenerator.getInstance().getTrajectorySet().level1StartToRocketFront, true));
                addSequential(new EjectHatch());
                addSequential(new WaitCommand("Eject Break", .45));
                addParallel(new IntakeHatch());

                //Turn
                // addSequential(new DriveMotionCommand(
                //                 TrajectoryGenerator.getInstance().getTrajectorySet().rocketFrontToTurn1A, false));

                // addParallel(new AutoCameraTrackWhenCrossXBoundary(70, MovingXDirection.Negative, 0.6)); // 100
                // addSequential(new DriveMotionCommand(
                //                 TrajectoryGenerator.getInstance().getTrajectorySet().rocketFrontTurn1AToLoading, false),
                //                 4);


                addSequential(new IntakeHatchArms(HatchArmState.IN));
                addSequential(new WaitCommand("Grab Break", .5));

                addSequential(new DriveMotionCommand(
                                TrajectoryGenerator.getInstance().getTrajectorySet().loadingToRocketBack, false));
                addParallel(new AutoTurn180CameraTrackWhenCrossXBoundary(290, MovingXDirection.Negative)); // 305
                addSequential(new DriveMotionCommand(
                                TrajectoryGenerator.getInstance().getTrajectorySet().turn3ToRocketBack, false));
                // addSequential(new WaitCommand("Eject Pause", .25));
                // addSequential(new EjectHatch());
        }
}
