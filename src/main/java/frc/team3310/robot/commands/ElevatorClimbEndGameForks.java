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
import frc.team3310.robot.subsystems.Climb.ForkShiftState;

public class ElevatorClimbEndGameForks extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ElevatorClimbEndGameForks() {
    addParallel(new ResetSensor());
    addSequential(new SetRobotClimbMode());
    addSequential(new ElevatorSetMMClimb(Constants.CLIMB_LVL_3));
    addSequential(new SetRobotLockedMode());
    addSequential(new WaitCommand("?", 3));
    addSequential(new SetRobotClimbFront());
    addSequential(new ElevatorSetMMClimb(1.0));
    addSequential(new SetRobotScoreMode());
    // addSequential(new ToggleForks(ForkShiftState.OUT));

  }
}
