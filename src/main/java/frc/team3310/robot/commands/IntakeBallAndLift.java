/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team3310.robot.subsystems.Elevator;
import frc.team3310.robot.subsystems.Intake;

public class IntakeBallAndLift extends CommandGroup {
  /**
   * Add your docs here.
   */
  public IntakeBallAndLift() {
    addSequential(new ElevatorSetPositionMP(Elevator.MIN_POSITION_INCHES));
    addSequential(new IntakeBallSensor(Intake.INTAKE_LOAD_SPEED));
    addSequential(new ElevatorSetPositionMP(Elevator.AFTER_INTAKE_POSITION_INCHES));


    
  }
}
