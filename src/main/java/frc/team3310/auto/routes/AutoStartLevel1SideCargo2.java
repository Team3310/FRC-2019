/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.auto.routes;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.team3310.auto.commands.AutoCameraTrackWhenCrossYBoundary;
import frc.team3310.auto.commands.AutoSpinMove180WhenCrossedXBoundary;
import frc.team3310.auto.commands.DriveMotionCommand;
import frc.team3310.auto.commands.WaitUntilCrossXBoundary.MovingXDirection;
import frc.team3310.auto.commands.WaitUntilCrossYBoundary.MovingYDirection;
import frc.team3310.robot.Constants;
import frc.team3310.robot.commands.EjectHatch;
import frc.team3310.robot.commands.ElevatorSetPositionMM;
import frc.team3310.robot.commands.ResetSensor;
import frc.team3310.robot.paths.TrajectoryGenerator;

public class AutoStartLevel1SideCargo2 extends CommandGroup {
        /**
         * Add your docs here.
         */
        public AutoStartLevel1SideCargo2() {
                addParallel(new ResetSensor());
                addParallel(new ElevatorSetPositionMM(Constants.AUTO_HATCH_LEVEL_1));
                addParallel(new AutoCameraTrackWhenCrossYBoundary(-75, MovingYDirection.OutsideToInside, 0.4,
                                Constants.finishedAtCargoLimeY));
                addSequential(new DriveMotionCommand(
                                TrajectoryGenerator.getInstance().getTrajectorySet().level1StartToCargoSide, true));
                addSequential(new WaitCommand("Eject Pause", .25));
                addSequential(new EjectHatch());

                addParallel(new AutoSpinMove180WhenCrossedXBoundary(135, MovingXDirection.Negative)); // 100

                addSequential(new DriveMotionCommand(
                                TrajectoryGenerator.getInstance().getTrajectorySet().cargoSideScoreMidToLoading,
                                false));
                // addSequential(new IntakeHatchArms(HatchArmState.IN));
                // addSequential(new WaitCommand("Grab Break", .25));
                // addSequential(new DriveMotionCommand(
                // TrajectoryGenerator.getInstance().getTrajectorySet().loadingToCargoSide,
                // false));
                // addSequential(new DriveSpinMove(-95));

                // addParallel(new AutoCameraTrackWhenCrossYBoundary(-58,
                // MovingYDirection.OutsideToInside, 0.7,
                // Constants.finishedAtCargoLimeY));
                // addSequential(new DriveMotionCommand(
                // TrajectoryGenerator.getInstance().getTrajectorySet().track2v2PoseToCargo2,
                // false));
                // addSequential(new WaitForChildren());
                // addSequential(new WaitCommand("Eject Pause", .25));
                // addSequential(new EjectHatch());
        }
}
