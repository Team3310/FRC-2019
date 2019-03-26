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

public class ElevatorSetMMClimb extends Command {
  private double targetPositionInches;
  // private boolean isAtTarget;
  // private static final double MIN_DELTA_TARGET = 0.3;

  public ElevatorSetMMClimb(double targetPositionInches) {
    this.targetPositionInches = targetPositionInches;
    requires(Robot.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // if (Math.abs(targetPositionInches - Robot.elevator.getClimbPositionInches())
    // < MIN_DELTA_TARGET) {
    Robot.elevator.setClimbMotionMagicPosition(targetPositionInches);
    System.out.println("Elevator set MP initialized, target = " + targetPositionInches);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (Elevator.getInstance().hasFinishedTrajectory()) {
      System.out.println("Trajectory finished end");
      return true;
    }
    if (Robot.elevator.getClimbFrontTop() || Elevator.getInstance().getClimbRearTop() || Robot.elevator.isClimbOverrided()) {
      System.out.println("Trajectory finished limit switch");
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  protected void end() {
    System.out.println("Elevator set MP end");
    // Robot.elevator.setJoystickPID();
    // Robot.elevator.resetEncoders();

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    System.out.println("ElevatorSetPositionMP interrupted");
    end();
    Robot.elevator.setJoystickPID();
    // Robot.elevator.resetEncoders();

  }
}
