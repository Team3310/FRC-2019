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

public class AutoTestTracking extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutoTestTracking() {
    // addParallel(new AutoCameraTrackWhenCrossedBoundary(50));
    // addSequential(new DriveMotionCommand(
    // TrajectoryGenerator.getInstance().getTrajectorySet().simpleStartToLeftSwitch,
    // true));
    addParallel(new ElevatorSetPositionMM(Constants.HATCH_LEVEL_1));
    addParallel(new AutoCameraTrackWhenCrossedBoundary(175));

    addSequential(new DriveMotionCommand(
        TrajectoryGenerator.getInstance().getTrajectorySet().level1StartToCargoFront, true));

    // addParallel(new AutoCameraTrackWhenCrossXBoundaryNegitive(-290)); //25

    addSequential(new EjectHatch());
    addSequential(new WaitCommand("Eject Break", 1));
    addSequential(
        new DriveMotionCommand(TrajectoryGenerator.getInstance().getTrajectorySet().cargoToTurn1, false));

    addParallel(new AutoCameraTrackWhenCrossXBoundaryNegative(35)); // 25

    addSequential(
        new DriveMotionCommand(TrajectoryGenerator.getInstance().getTrajectorySet().turn1ToLoading, false));

  }
}
