package frc.team3310.robot.commands;

import frc.team3310.robot.Constants;
import frc.team3310.robot.Robot;
import frc.team3310.robot.subsystems.Drive.DriveControlMode;

public class DrivePathCameraTrack extends ExtraTimeoutCommand {

	private boolean isTracking = false;
	private double velocityScale = 1.0;
	private final double PIPELINE_TIMEOUT = 0.2;
	private final double isVisonTimedOut = 1.5;
	private boolean isTrackFinished = false;
	private double finishAtLimeY = Constants.finishedAtCargoLimeY;
	private int invalidCounter = 0;

	public DrivePathCameraTrack() {
		requires(Robot.drive);
	}

	public DrivePathCameraTrack(double velocityScale, double finishAtLimeY) {
		this.velocityScale = velocityScale;
		this.finishAtLimeY = finishAtLimeY;
		requires(Robot.drive);
		isTrackFinished = false;
		invalidCounter = 0;
	}

	protected void initialize() {
		System.out.println("Switch Pipeline");
		isTracking = false;
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
			System.out.println("Start camera track");
			System.out.println("Velocity Scale " + velocityScale);
			Robot.drive.setCameraTrack(velocityScale);
			isTracking = true;
		} 
		else if (isTracking && !isTrackFinished) {
			if (Robot.drive.isLimeValid == false) {
				invalidCounter++;
			}
			else {
				invalidCounter = 0;
			}

			isTrackFinished = (Robot.drive.isLimeValid == false && invalidCounter > 10) || (Robot.drive.limeY < finishAtLimeY); // || isExtraTwoTimedOut();
			
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
		System.out.println("camera track finished");
		System.out.println("Time to eject done camera track");
		Robot.drive.setControlMode(DriveControlMode.JOYSTICK);
		Robot.drive.overrideTrajectory(true);
		Robot.drive.setPipeline(1);
		Robot.drive.setSpeed(0.0);
	}

	protected void interrupted() {
		System.out.println("camera track interrupted");
		System.out.println("Time to eject done camera track");
		end();
	}
}