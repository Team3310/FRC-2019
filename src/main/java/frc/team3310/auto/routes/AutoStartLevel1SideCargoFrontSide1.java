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
import frc.team3310.auto.commands.DriveMotionCommand;
import frc.team3310.auto.commands.WaitUntilCrossXBoundary.MovingXDirection;
import frc.team3310.auto.commands.WaitUntilCrossYBoundary.MovingYDirection;
import frc.team3310.robot.Constants;
import frc.team3310.robot.commands.EjectHatch;
import frc.team3310.robot.commands.ElevatorSetPositionMM;
import frc.team3310.robot.commands.IntakeHatch;
import frc.team3310.robot.commands.IntakeHatchArms;
import frc.team3310.robot.paths.TrajectoryGenerator;
import frc.team3310.robot.subsystems.Intake.HatchArmState;

public class AutoStartLevel1SideCargoFrontSide1 extends CommandGroup {

        public AutoStartLevel1SideCargoFrontSide1() {
                addParallel(new ElevatorSetPositionMM(Constants.HATCH_LEVEL_1));

                addParallel(new AutoCameraTrackWhenCrossXBoundary(175, MovingXDirection.Positive, 1.0,
                                Constants.finishedAtCargoLimeY));
                addSequential(new DriveMotionCommand(
                                TrajectoryGenerator.getInstance().getTrajectorySet().level1StartToCargoFront, true));
                addSequential(new EjectHatch());
                addSequential(new WaitCommand("Eject Break", .25));
                addParallel(new IntakeHatch());
                addSequential(new DriveMotionCommand(
                                TrajectoryGenerator.getInstance().getTrajectorySet().cargoFrontToTurn1, false));

                addParallel(new AutoCameraTrackWhenCrossXBoundary(85, MovingXDirection.Negative, 0.7,
                                Constants.finishedAtCargoLimeY));
                addSequential(new DriveMotionCommand(
                                TrajectoryGenerator.getInstance().getTrajectorySet().cargoFrontTurn1ToLoading, false),
                                4);
                addSequential(new IntakeHatchArms(HatchArmState.IN));
                addSequential(new WaitCommand("Grab Break", .15));

                // addSequential(new DriveMotionCommand(
                // TrajectoryGenerator.getInstance().getTrajectorySet().loadingToTurnCargoSide,
                // false));
                // addParallel(new AutoCameraTrackWhenCrossYBoundary(-75,
                // MovingYDirection.OutsideToInside, 0.6, Constants.finishedAtCargoLimeY));
                // addSequential(
                // new
                // DriveMotionCommand(TrajectoryGenerator.getInstance().getTrajectorySet().turn1ToCargoSide1,
                // false));
                // addSequential(new WaitCommand("Eject Pause", .25));
                // addSequential(new EjectHatch());
        }
}
