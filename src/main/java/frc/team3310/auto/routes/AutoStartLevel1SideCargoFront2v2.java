/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.auto.routes;

import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitForChildren;
import frc.team3310.auto.commands.AutoCameraTrackWhenCrossXBoundary;
import frc.team3310.auto.commands.DriveMotionCommand;
import frc.team3310.auto.commands.LazyLoadCommandGroup;
import frc.team3310.auto.commands.WaitUntilCrossXBoundary.MovingXDirection;
import frc.team3310.robot.Constants;
import frc.team3310.robot.commands.EjectHatch;
import frc.team3310.robot.commands.ElevatorSetPositionMM;
import frc.team3310.robot.commands.IntakeHatch;
import frc.team3310.robot.commands.IntakeHatchArms;
import frc.team3310.robot.paths.TrajectoryGenerator;
import frc.team3310.robot.subsystems.Intake.HatchArmState;

public class AutoStartLevel1SideCargoFront2v2 extends LazyLoadCommandGroup {
        /**
         * Add your docs here.
         */
        public AutoStartLevel1SideCargoFront2v2() {
                addParallel(new ElevatorSetPositionMM(Constants.AUTO_HATCH_LEVEL_1));

                addParallel(new AutoCameraTrackWhenCrossXBoundary(175, MovingXDirection.Positive, 1.0,
                                Constants.finishedAtCargoLimeY));
                addSequential(new DriveMotionCommand(registerTrajectory(
                                TrajectoryGenerator.getInstance().getTrajectorySet().level1StartToCargoFront), true));
                addSequential(new WaitForChildren());
                addSequential(new EjectHatch());
                addSequential(new WaitCommand("Eject Break", .25));
                addParallel(new IntakeHatch());
                addSequential(new DriveMotionCommand(
                                registerTrajectory(
                                                TrajectoryGenerator.getInstance().getTrajectorySet().cargoFrontToTurn1),
                                false));

                addParallel(new AutoCameraTrackWhenCrossXBoundary(85, MovingXDirection.Negative, 0.6,
                                Constants.finishedAtCargoLimeY));
                addSequential(new DriveMotionCommand(registerTrajectory(
                                TrajectoryGenerator.getInstance().getTrajectorySet().cargoFrontTurn1ToLoading), false),
                                4);
                addSequential(new WaitForChildren());
                addSequential(new IntakeHatchArms(HatchArmState.IN));
                addSequential(new WaitCommand("Grab Break", .25));

                addSequential(new DriveMotionCommand(registerTrajectory(
                                TrajectoryGenerator.getInstance().getTrajectorySet().loadingToCargoFrontTrack2v2),
                                false));
                addParallel(new AutoCameraTrackWhenCrossXBoundary(170, MovingXDirection.Positive, 1.0,
                                Constants.finishedAtCargoLimeY));
                addSequential(new DriveMotionCommand(registerTrajectory(
                                TrajectoryGenerator.getInstance().getTrajectorySet().track2v2PoseToCargo2), false));
                // addSequential(new WaitForChildren());
                // addSequential(new WaitCommand("Eject Pause", .25));
                // addSequential(new EjectHatch());
        }
}
