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

public class ElevatorClimbEndGameSuccLvl3 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ElevatorClimbEndGameSuccLvl3() {
    addParallel(new ResetSensor());
    addSequential(new SetRobotClimbFront());
    addSequential(new ElevatorSetMMClimb(Constants.CLIMB_LIFT_FRONT));
    addParallel(new ResetSensor());
    addParallel(new TurnClimbPumpOn());
    addSequential(new SetRobotClimbMode());
    addSequential(new ElevatorSetMMClimb(Constants.CLIMB_LVL_3_SUCC));
    addSequential(new WaitCommand("Succ Pause", 3));
    addSequential(new SetRobotClimbMode());
    addSequential(new ElevatorSetMMClimb(3.0));
  }
}