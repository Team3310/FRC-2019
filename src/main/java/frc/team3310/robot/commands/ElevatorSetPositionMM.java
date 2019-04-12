/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot.commands;

import frc.team3310.robot.Constants;
import frc.team3310.robot.Robot;
import frc.team3310.robot.subsystems.Elevator;

public class ElevatorSetPositionMM extends ExtraTimeoutCommand {
  private double targetPositionInches;

  public ElevatorSetPositionMM(double targetPositionInches) {
    this.targetPositionInches = targetPositionInches;
    requires(Robot.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    // if (Robot.elevator.elevatorCargoMode == true) {
    //   Robot.elevator.setElevatorMotionMagicPosition(targetPositionInches + Constants.BALL_OFFSET);

    // } else {
    //   Robot.elevator.setElevatorMotionMagicPosition(targetPositionInches);
    // }
    Robot.elevator.setElevatorMotionMagicPosition(targetPositionInches);

    resetExtraOneTimer();
    startExtraOneTimeout(0.1);
    // System.out.println("Elevator set MM initialized, target = " + targetPositionInches);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // System.out.println("Elevator position = " + Robot.elevator.getElevatorInchesOffGround());
    if (isExtraOneTimedOut() && Elevator.getInstance().hasFinishedTrajectory()) {
      // System.out.println("Trajectory finished");
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
