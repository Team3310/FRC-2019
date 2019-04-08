package frc.team3310.auto.commands;

import frc.team3310.robot.Constants;
import frc.team3310.robot.Robot;
import frc.team3310.robot.commands.ExtraTimeoutCommand;
import frc.team3310.robot.subsystems.Drive.DriveControlMode;

public class DrivePathCameraTrack extends ExtraTimeoutCommand {

	private boolean isTracking = false;
	private double velocityScale = 1.0;
	private final double PIPELINE_TIMEOUT = 0.02;
	private boolean isTrackFinished = false;
	private double finishAtLimeY = Constants.finishedAtCargoLimeY;
	private int invalidCounter = 0;
	private int velocityWindowSize = 25;
	private int windowIndex = 0;
	private boolean windowFull = false;
	private double velocitySum = 0;
	private double lastLimeY = 0;
	private double[] pitchAverageWindow = new double[velocityWindowSize];

	public DrivePathCameraTrack() {
		requires(Robot.drive);
	}

	public DrivePathCameraTrack(double velocityScale, double finishAtLimeY) {
		this.velocityScale = velocityScale;
		this.finishAtLimeY = finishAtLimeY;
		requires(Robot.drive);
	}

	protected void initialize() {
		System.out.println("Switch Pipeline");
		isTrackFinished = false;
		isTracking = false;
		windowFull = false;
		invalidCounter = 0;
		resetExtraOneTimer();
		resetExtraTwoTimer();
		startExtraOneTimeout(PIPELINE_TIMEOUT);
		Robot.drive.setPipeline(0);
		Robot.drive.isLimeValid = true;
	}

	protected void execute() {
		if (!isTracking && isExtraOneTimedOut()) {
			System.out.println("Start camera track");
			System.out.println("Velocity Scale " + velocityScale);
			Robot.drive.setCameraTrack(velocityScale);
			if (Robot.drive.isLimeValid) {
				lastLimeY = Robot.drive.limeY;
			}
			else {
				lastLimeY = -9999;
			}
			isTracking = true;
		} else if (isTracking && !isTrackFinished) {

			if (Robot.drive.isLimeValid == false) {
				invalidCounter++;
			} else {
				invalidCounter = 0;
			}

			// if (Robot.drive.isLimeValid ) {
				// if (lastLimeY < -9998.0) {
				// 	lastLimeY = Robot.drive.limeY;
				// }
				// double avgChangeInLimeY = updatePitchWindow();
				// lastLimeY = Robot.drive.limeY;
				// if (windowFull && avgChangeInLimeY < Math.abs(0.2)) {
				// 	isTrackFinished = true;
				// 	System.out.println("No move finished");
				// }			
				// else {
		
					isTrackFinished = (Robot.drive.isLimeValid == false && invalidCounter > 30)
							|| (Robot.drive.limeY < finishAtLimeY); // || isExtraTwoTimedOut();
				// }
				// System.out.println("Avg limeY = " + avgChangeInLimeY + ", valid = " + Robot.drive.isLimeValid + ", lime Y =" + Robot.drive.limeY);
			// }

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

	private double updatePitchWindow() {
		double lastPitchAngle = pitchAverageWindow[windowIndex];
		double currentPitchAngle = Robot.drive.limeY - lastLimeY;
		pitchAverageWindow[windowIndex] = currentPitchAngle;
		velocitySum = velocitySum - lastPitchAngle + currentPitchAngle;

		windowIndex++;
		if (windowIndex == velocityWindowSize) {
			windowFull = true;
			windowIndex = 0;
		}

		return velocitySum / velocityWindowSize;
	}

}