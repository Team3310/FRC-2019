package frc.team3310.auto.commands;

import frc.team3310.robot.Constants;
import frc.team3310.robot.Robot;
import frc.team3310.robot.commands.ExtraTimeoutCommand;
import frc.team3310.robot.subsystems.Drive.DriveControlMode;

public class DrivePathCameraTrackWithVelocity extends ExtraTimeoutCommand {

  private boolean isTracking = false;
  private double velocityScale = 1.0;
  private final double PIPELINE_TIMEOUT = 0.02;
  // private final double isVisonTimedOut = 1.5;
  private boolean isTrackFinished = false;
  private double ultraSonicDistance = Constants.finishedAtLoadingUlt;
  private double finishAtLimeY = Constants.finishedAtCargoLimeY;
  private int invalidCounter = 0;

  public DrivePathCameraTrackWithVelocity() {
    requires(Robot.drive);
  }

  public DrivePathCameraTrackWithVelocity(double velocityScale, double finishAtLimeY, double ultraSonicDistance) {
    this.velocityScale = velocityScale;
    this.finishAtLimeY = finishAtLimeY;
    this.ultraSonicDistance = ultraSonicDistance;
    requires(Robot.drive);
    isTrackFinished = false;
    invalidCounter = 0;
  }

  protected void initialize() {
    // System.out.println("Switch Pipeline");
    isTracking = false;
    invalidCounter = 0;
    resetExtraOneTimer();
    resetExtraTwoTimer();
    startExtraOneTimeout(PIPELINE_TIMEOUT);
    // startExtraTwoTimeout(isVisonTimedOut);
    Robot.drive.setPipeline(0);

    // setTimeout(timeout);
    Robot.drive.isLimeValid = true;
  }

  protected void execute() {
    if (!isTracking && isExtraOneTimedOut()) {
      // System.out.println("Start camera track");
      // System.out.println("Velocity Scale " + velocityScale);
      Robot.drive.setCameraTrackWithVelocity(velocityScale);
      isTracking = true;
    } else if (isTracking && !isTrackFinished) {
      if (Robot.drive.isLimeValid == false) {
        invalidCounter++;
      } else {
        invalidCounter = 0;
      }

      isTrackFinished = (Robot.drive.isLimeValid == false && invalidCounter > 50)
          || (Robot.drive.limeY < finishAtLimeY) || Robot.drive.getUltrasonicDistance() <= ultraSonicDistance; // || isExtraTwoTimedOut();

      // isTrackFinished = Robot.drive.getUltrasonicDistance() < Constants.finishedAtCargoUlt;

      System.out.println("valid = " + Robot.drive.isLimeValid + ", lime Y =" + Robot.drive.limeY);
      if (isTrackFinished == true) {
        startExtraTwoTimeout(0.2);
        Robot.drive.setSpeed(0.2);
      }
    }
  }

  protected boolean isFinished() {
    return isExtraTwoTimedOut();
  }

  protected void end() {
    // System.out.println("camera track finished");
    // System.out.println("Time to eject done camera track");
    Robot.drive.setControlMode(DriveControlMode.JOYSTICK);
    Robot.drive.overrideTrajectory(true);
    Robot.drive.setPipeline(1);
    Robot.drive.setSpeed(0.0);
  }

  protected void interrupted() {
    // System.out.println("camera track interrupted");
    // System.out.println("Time to eject done camera track");
    end();
  }
}