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
import frc.team3310.robot.subsystems.Climb.ArmShiftState;
import frc.team3310.robot.subsystems.Climb.ForkShiftState;

public class ElevatorClimbEndGameForks extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ElevatorClimbEndGameForks() {
    addParallel(new ResetSensor());
    addSequential(new SetRobotClimbFront());
    addSequential(new ElevatorSetMMClimb(.75));
    addParallel(new ResetSensor());
    addSequential(new SetRobotClimbMode());
    addSequential(new ElevatorSetMMClimb(Constants.CLIMB_LVL_3_FORKS));
    addParallel(new WaitCommand("Forks Deploy", .5));
    addSequential(new ToggleForks(ForkShiftState.OUT));
    addSequential(new SetRobotClimbBack());
    addSequential(new ElevatorSetMMClimb(Constants.CLIMB_LVL_3_FORKS + 2));// 1.5
    addSequential(new DriveForwardClimbMM(10, false));
    addSequential(new SetRobotClimbFront());
    addSequential(new ElevatorSetMMClimb(0.0));

  }
}
