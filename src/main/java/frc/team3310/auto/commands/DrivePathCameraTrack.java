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
	private double velocitySum = 0;
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
			isTracking = true;
		} else if (isTracking && !isTrackFinished) {
			// double avgVelocity = updatePitchWindow();
			// if (windowIndex > velocityWindowSize && avgVelocity < 500) {
			// 	isTrackFinished = true;
			// }
			// else {
				if (Robot.drive.isLimeValid == false) {
					invalidCounter++;
				} else {
					invalidCounter = 0;
				}
	
				isTrackFinished = (Robot.drive.isLimeValid == false && invalidCounter > 10)
						|| (Robot.drive.limeY < finishAtLimeY); // || isExtraTwoTimedOut();
			// }

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

	private double updatePitchWindow() {
		double lastPitchAngle = pitchAverageWindow[windowIndex];
		double currentPitchAngle = Robot.drive.getAverageRightLeftVelocity();
		pitchAverageWindow[windowIndex] = currentPitchAngle;
		velocitySum = velocitySum - lastPitchAngle + currentPitchAngle;

		windowIndex++;
		if (windowIndex == velocityWindowSize) {
			windowIndex = 0;
		}

		return velocitySum / velocityWindowSize;
	}

}