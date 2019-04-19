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

public class ElevatorClimbEndGameSuccLvl3 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ElevatorClimbEndGameSuccLvl3() {
    addParallel(new ResetSensor());
    addSequential(new SetRobotClimbMode());
    addSequential(new WaitCommand("Shift Pause", .10));
    addParallel(new TurnClimbPumpOn());
    addSequential(new ElevatorSetMMClimb(Constants.CLIMB_LVL_3_SUCC));
    addSequential(new WaitCommand("Shift Pause", .10));
    addParallel(new ToggleSuccArm(ArmShiftState.OUT));
    addSequential(new WaitCommand("Arm Settle Pause", 1));
    addSequential(new SetRobotClimbFront());
    addSequential(new WaitCommand("Arm Settle Pause", .10));
    addSequential(new ElevatorSetMMClimb(Constants.CLIMB_LVL_3_SUCC + 1.75, 3000, 7000));//1.5  normal vel = 8000, accel = 15000
    addSequential(new SetRobotLockedMode());
    // addSequential(new SetRobotClimbMode());
    // addSequential(new WaitCommand("Shift Pause", .10));
    // addSequential(new ElevatorSetMMClimb(3.0));
  }
}
