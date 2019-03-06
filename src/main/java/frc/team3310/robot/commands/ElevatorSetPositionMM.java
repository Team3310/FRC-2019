/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.Robot;
import frc.team3310.robot.subsystems.Elevator;

public class ElevatorSetPositionMM extends Command {
  private double targetPositionInches;
  private boolean isAtTarget;
  private static final double MIN_DELTA_TARGET = 0.3;

  public ElevatorSetPositionMM(double targetPositionInches) {
    this.targetPositionInches = targetPositionInches;
    requires(Robot.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // if (Math.abs(targetPositionInches -
    // Robot.elevator.getElevatorPositionInches()) < MIN_DELTA_TARGET) {
    // isAtTarget = true;
    // } else {
    // isAtTarget = false;
    Robot.elevator.setElevatorMotionMagicPosition(targetPositionInches);
    // }
    System.out.println("Elevator set MM initialized, target = " + targetPositionInches);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (Elevator.getInstance().hasFinishedTrajectory()) {
      System.out.println("Trajectory finished");
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  protected void end() {
    Robot.elevator.setJoystickPID();
    System.out.println("Elevator set MP end");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    System.out.println("ElevatorSetPositionMP interrupted");
    end();
    // Robot.elevator.setPositionPID(Robot.elevator.getPositionInches());
  }
}
