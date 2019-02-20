/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team3310.robot.subsystems.Intake;
import frc.team3310.robot.subsystems.Intake.BallArmState;

public class IntakeBallHold extends CommandGroup {
  /**
   * Add your docs here.
   */
  public IntakeBallHold() {
    addParallel(new IntakeBallArms(BallArmState.IN));
    addParallel(new IntakeSetSpeed(Intake.INTAKE_HOLD_SPEED));
    addSequential(new ElevatorBallLevel());
  }
}
