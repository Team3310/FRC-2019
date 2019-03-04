/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team3310.robot.Constants;

public class ElevatorClimbEndGamev2 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ElevatorClimbEndGamev2() {
    addParallel(new ResetSensor());
    addSequential(new SetRobotClimbMode());
    addSequential(new ElevatorSetMMClimb(Constants.CLIMB));
    addParallel(new DriveRaiseFrontLegOnSensor());
    addSequential(new DriveForwardClimbMM(30, true));
    addSequential(new SetRobotClimbBack());
    addSequential(new ElevatorSetMMClimb(-(Constants.CLIMB - 0.5)));
    addSequential(new SetRobotScoreMode());
  }
}
