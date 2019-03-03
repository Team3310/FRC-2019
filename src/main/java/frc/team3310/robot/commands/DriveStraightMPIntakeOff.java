package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.Robot;
import frc.team3310.robot.subsystems.Drive.DriveControlMode;

public class DriveStraightMPIntakeOff extends Command {
	private double distanceInches, maxVelocityInchesPerSec, desiredAbsoluteAngle;
	private boolean useGyroLock, useAbsolute;
	private double speed;

	public DriveStraightMPIntakeOff(double speed, double distanceInches, double maxVelocityInchesPerSec,
			boolean useGyroLock, boolean useAbsolute, double desiredAbsoluteAngle) {
		requires(Robot.drive);
		this.distanceInches = distanceInches;
		this.maxVelocityInchesPerSec = maxVelocityInchesPerSec;
		this.desiredAbsoluteAngle = desiredAbsoluteAngle;
		this.useGyroLock = useGyroLock;
		this.useAbsolute = useAbsolute;
		this.speed = speed;
	}

	protected void initialize() {
		Robot.drive.driveForwardClimb(speed);
		Robot.drive.setStraightMP(distanceInches, maxVelocityInchesPerSec, useGyroLock, useAbsolute,
				desiredAbsoluteAngle);
	}

	protected void execute() {
	}

	protected boolean isFinished() {
		return Robot.elevator.getPlatformDetectRear();
	}

	protected void end() {
		Robot.drive.setControlMode(DriveControlMode.JOYSTICK);
		Robot.drive.driveForwardClimb(0);
	}

	protected void interrupted() {
		// System.out.println("DriveStraightMPIntakeOff interrupted");
		end();
	}
}