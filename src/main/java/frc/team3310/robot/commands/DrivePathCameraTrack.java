package frc.team3310.robot.commands;

import frc.team3310.robot.Robot;
import frc.team3310.robot.subsystems.Drive.DriveControlMode;

public class DrivePathCameraTrack extends ExtraTimeoutCommand {

	private boolean isTracking = false;
	private double velocityScale = 1.0;
	private final double PIPELINE_TIMEOUT = 0.2;
	private final double isVisonTimedOut = 1.5;
	private boolean isTrackFinished = false;

	public DrivePathCameraTrack() {
		requires(Robot.drive);
	}

	public DrivePathCameraTrack(double velocityScale) {
		this.velocityScale = velocityScale;
		requires(Robot.drive);
		isTrackFinished = false;
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
			Robot.drive.setCameraTrack(velocityScale);
			isTracking = true;
		} else if (!isTrackFinished) {
			isTrackFinished = Robot.drive.isLimeValid == false || Robot.drive.limeY < -5; // || isExtraTwoTimedOut();
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