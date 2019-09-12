package frc.team3310.auto.commands;

import frc.team3310.robot.Constants;
import frc.team3310.robot.Robot;
import frc.team3310.robot.commands.ExtraTimeoutCommand;

public class DrivePathCameraTrack extends ExtraTimeoutCommand {

	private boolean isTracking = false;
	private double velocityScale = 1.0;
	private final double PIPELINE_TIMEOUT = 0.02;
	private boolean isTrackFinished = false;
	private double finishAtLimeY = Constants.finishedAtCargoLimeY;
	private double ultraSonicDistance = Constants.finishedAtLoadingUlt;
	private int invalidCounter = 0;

	public DrivePathCameraTrack() {
		requires(Robot.drive);
	}

	public DrivePathCameraTrack(double velocityScale, double finishAtLimeY, double ultraSonicDistance) {
		this.velocityScale = velocityScale;
		this.finishAtLimeY = finishAtLimeY;
		this.ultraSonicDistance = ultraSonicDistance;
		requires(Robot.drive);
	}

	protected void initialize() {
		// System.out.println("Switch Pipeline");
		isTrackFinished = false;
		invalidCounter = 0;
		resetExtraOneTimer();
		resetExtraTwoTimer();
		startExtraOneTimeout(PIPELINE_TIMEOUT);
		Robot.drive.setPipeline(2);
		Robot.drive.isLimeValid = true;
	}

	protected void execute() {
		if (!isTracking && isExtraOneTimedOut()) {
			// System.out.println("Start camera track");
			// System.out.println("Velocity Scale " + velocityScale);
			Robot.drive.setCameraTrack(velocityScale);
			isTracking = true;

		} else if (isTracking && !isTrackFinished) {

			if (Robot.drive.isLimeValid == false) {
				invalidCounter++;
			} else {
				invalidCounter = 0;
			}
		
			 isTrackFinished = (Robot.drive.limeY < finishAtLimeY) || Robot.drive.getUltrasonicDistance() <= ultraSonicDistance;
							 
			// isTrackFinished = Robot.drive.getUltrasonicDistance() < Constants.finishedAtCargoUlt;
			//(Robot.drive.isLimeValid == false && invalidCounter > 50)
	
			// System.out.println(", valid = " + Robot.drive.isLimeValid + ", Ultrasonic Distance =" + Robot.drive.getUltrasonicDistance());

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
		Robot.drive.overrideTrajectory(true);
		Robot.drive.setPipeline(1);
		Robot.drive.setSpeed(0.0);
	}

	protected void interrupted() {
		// System.out.println("camera track interrupted");
		Robot.drive.overrideTrajectory(true);
		Robot.drive.setPipeline(1);
		Robot.drive.setSpeed(0.0);	

	}

}