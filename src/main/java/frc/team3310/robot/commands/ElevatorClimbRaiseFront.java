/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ElevatorClimbRaiseFront extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ElevatorClimbRaiseFront() {
    addSequential(new SetRobotClimbFront());
    addSequential(new ElevatorSetMMClimb(1.0));
    addSequential(new SetRobotClimbMode());
  }
}
