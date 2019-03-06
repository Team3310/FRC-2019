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
import frc.team3310.robot.subsystems.Intake.HatchArmState;
import frc.team3310.utility.lib.geometry.Translation2d;

public class AutoStartLevel1SideCargoFrontSide1 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutoStartLevel1SideCargoFrontSide1() {
    addParallel(new ElevatorSetPositionMM(Constants.HATCH_LEVEL_1));

    addParallel(new AutoCameraTrackWhenCrossedBoundary(175));
    addSequential(
        new DriveMotionCommand(TrajectoryGenerator.getInstance().getTrajectorySet().level1StartToCargoFront, true));
    addSequential(new EjectHatch());
    addSequential(new WaitCommand("Eject Break", .25));
    addParallel(new IntakeHatch());
    addSequential(
        new DriveMotionCommand(TrajectoryGenerator.getInstance().getTrajectorySet().cargoFrontToTurn1A, false));

    addParallel(new AutoCameraTrackWhenCrossXBoundaryNegative(60, 0.7));
    addSequential(
        new DriveMotionCommand(TrajectoryGenerator.getInstance().getTrajectorySet().cargoFrontTurn1AToLoading, false),
        4);
    addSequential(new IntakeHatchArms(HatchArmState.IN));
    addSequential(new WaitCommand("Grab Break", .25));

    addSequential(
        new DriveMotionCommand(TrajectoryGenerator.getInstance().getTrajectorySet().loadingToTurn1CargoSide1, false));
    addParallel(new AutoCameraTrackWhenInRegion(new Translation2d(245, -22.5), new Translation2d(276.5, -70)));
    addSequential(
        new DriveMotionCommand(TrajectoryGenerator.getInstance().getTrajectorySet().turn1ToCargoSide1, false));
    addSequential(new WaitCommand("Eject Pause", .25));
    addSequential(new EjectHatch());
  }
}
