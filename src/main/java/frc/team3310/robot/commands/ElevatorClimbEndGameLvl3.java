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

public class ElevatorClimbEndGameLvl3 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ElevatorClimbEndGameLvl3() {
    addParallel(new ResetSensor());
    addSequential(new SetRobotClimbMode());
    addSequential(new ElevatorSetMMClimb(Constants.CLIMB_LVL_3));
    addSequential(new DriveForwardClimbMM(10, false));
    addSequential(new SetRobotClimbFront());
    addSequential(new ElevatorSetMMClimb(0.0));
    addSequential(new DriveForwardClimbMM(10, true));
    addSequential(new SetRobotClimbBack());
    addSequential(new ElevatorSetMMClimb(-(Constants.CLIMB_LVL_3)));
    addSequential(new SetRobotScoreMode());
    addSequential(new DriveSetSpeed(0.3, 0.3));
  }
}
