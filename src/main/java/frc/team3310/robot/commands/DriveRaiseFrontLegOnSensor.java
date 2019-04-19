package frc.team3310.robot.commands;

import frc.team3310.robot.Robot;

/**
 *
 */
public class DriveRaiseFrontLegOnSensor extends ExtraTimeoutCommand {

  private boolean edgeDetected = false;
  private boolean frontRaised = false;

  public DriveRaiseFrontLegOnSensor() {
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    edgeDetected = false;
    frontRaised = false;
    resetExtraOneTimer();
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
    if (edgeDetected == false && Robot.elevator.getPlatformDetectFront()) {
      startExtraOneTimeout(0.4);
      edgeDetected = true;

    }
    if (isExtraOneTimedOut()) {
      Robot.elevator.setRobotClimbFront();
      Robot.elevator.setClimbMotionMagicPosition(0);
      // System.out.println("Found sensor edge");
      frontRaised = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    return Robot.elevator.hasFinishedTrajectory() && edgeDetected == true && isExtraOneTimedOut()
        && frontRaised == true;
  }

  // Called once after isFinished returns true
  protected void end() {
    // System.out.println("Finished Edge");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    end();
  }
}
