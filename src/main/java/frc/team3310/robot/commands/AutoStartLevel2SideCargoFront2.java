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

public class AutoStartLevel2SideCargoFront2 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutoStartLevel2SideCargoFront2() {
    addParallel(new ElevatorSetPositionMM(Constants.HATCH_LEVEL_1));

    addParallel(new AutoCameraTrackWhenCrossXBoundary(175, MovingXDirection.Positive));
    addSequential(
        new DriveMotionCommand(TrajectoryGenerator.getInstance().getTrajectorySet().level2StartToCargoFront, true));
    addSequential(new EjectHatch());
    addSequential(new WaitCommand("Eject Break", .25));
    addParallel(new ElevatorAutoZeroSensor());
    addParallel(new AutoTurn180CameraTrackWhenCrossXBoundary(95, MovingXDirection.Negative, 0.7));
    addSequential(new DriveMotionCommand(
        TrajectoryGenerator.getInstance().getTrajectorySet().cargoFrontToLoadingReversed, false));
    addSequential(new WaitCommand("Turn Break", 2));
    addSequential(new IntakeHatch());
    addSequential(new DrivePathCameraTrack(1));

  }
}
