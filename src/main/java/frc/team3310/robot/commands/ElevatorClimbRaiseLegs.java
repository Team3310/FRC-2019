/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ElevatorClimbRaiseLegs extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ElevatorClimbRaiseLegs() {
    addSequential(new ResetSensor());
    addSequential(new SetRobotClimbBack());
    addSequential(new ElevatorSetMMClimb(-20));
    addSequential(new SetRobotLockedMode());

  }
}
