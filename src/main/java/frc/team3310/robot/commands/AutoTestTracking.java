/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.team3310.robot.paths.TrajectoryGenerator;
import frc.team3310.robot.Constants;
import frc.team3310.robot.commands.WaitUntilCrossXBoundary.MovingXDirection;

public class AutoTestTracking extends CommandGroup {
 
  public AutoTestTracking() {
    // addParallel(new AutoCameraTrackWhenCrossedBoundary(50));
    // addSequential(new DriveMotionCommand(
    // TrajectoryGenerator.getInstance().getTrajectorySet().simpleStartToLeftSwitch,
    // true));
    addParallel(new ElevatorSetPositionMM(Constants.HATCH_LEVEL_1));
    addParallel(new AutoCameraTrackWhenCrossXBoundary(175, MovingXDirection.Positive));

    addSequential(new DriveMotionCommand(TrajectoryGenerator.getInstance().getTrajectorySet().level1StartToCargoFront, true));

    // addParallel(new AutoCameraTrackWhenCrossXBoundaryNegitive(-290)); //25

    addSequential(new EjectHatch());
    addSequential(new WaitCommand("Eject Break", 1));
    addSequential(new DriveMotionCommand(TrajectoryGenerator.getInstance().getTrajectorySet().cargoFrontToTurn1, false));

    addParallel(new AutoCameraTrackWhenCrossXBoundary(35, MovingXDirection.Negative)); // 25

    addSequential(new DriveMotionCommand(TrajectoryGenerator.getInstance().getTrajectorySet().cargoFrontTurn1ToLoading, false));
  }
}
