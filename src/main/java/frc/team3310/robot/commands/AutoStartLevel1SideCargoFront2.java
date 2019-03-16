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

public class AutoStartLevel1SideCargoFront2 extends CommandGroup {
        /**
         * Add your docs here.
         */
        public AutoStartLevel1SideCargoFront2() {
                addParallel(new ElevatorSetPositionMM(Constants.HATCH_LEVEL_1));

                addParallel(new AutoTurn180CameraTrackWhenCrossXBoundary(175, MovingXDirection.Positive));
                addSequential(new DriveMotionCommand(
                                TrajectoryGenerator.getInstance().getTrajectorySet().level1StartToCargoFront, true));
                // addSequential(new EjectHatch());
                addSequential(new WaitCommand("Eject Break", .25));
                // addParallel(new IntakeHatch());
                addSequential(new DriveMotionCommand(
                                TrajectoryGenerator.getInstance().getTrajectorySet().cargoFrontToLoadingReversed,
                                false));

                // addParallel(new AutoCameraTrackWhenCrossXBoundary(85,
                // MovingXDirection.Negative, 0.7));
                // addSequential(new DriveMotionCommand(
                // TrajectoryGenerator.getInstance().getTrajectorySet().cargoFrontTurn1ToLoading,
                // false), 4);
                // addSequential(new IntakeHatchArms(HatchArmState.IN));

                addSequential(new WaitCommand("Grab Break", .25));

                // addSequential(new DriveMotionCommand(
                //                 TrajectoryGenerator.getInstance().getTrajectorySet().loadingTocargoFrontTrack2, false));
                // addParallel(new AutoCameraTrackWhenCrossXBoundary(170,
                // MovingXDirection.Positive));
                // addSequential(new DriveMotionCommand(
                //                 TrajectoryGenerator.getInstance().getTrajectorySet().track2PoseToCargo2, false));
                // addSequential(new WaitCommand("Eject Pause", .25));
                // addSequential(new EjectHatch());
        }
}
