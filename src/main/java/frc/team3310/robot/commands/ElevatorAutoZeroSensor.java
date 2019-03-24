/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.Constants;
import frc.team3310.robot.Robot;
import frc.team3310.robot.subsystems.Elevator;
import frc.team3310.robot.subsystems.Intake.BallArmState;
import frc.team3310.robot.subsystems.Intake.HatchArmState;

public class ElevatorAutoZeroSensor extends Command {

  public ElevatorAutoZeroSensor() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.elevator.setSpeed(Elevator.AUTO_ZERO_SPEED);
    System.out.println("Auto zero initialize");
    Robot.intake.setBallArmState(BallArmState.IN);
    Robot.intake.setHatchArmState(HatchArmState.IN);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.elevator.getMinElevatorSensor();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.elevator.setSpeed(0);
    Robot.elevator.resetEncoders(Constants.LOW_HOME_POSITION_INCHES);
    Robot.elevator.setJoystickOpenLoop();
    System.out.println("Elevator Zeroed");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
