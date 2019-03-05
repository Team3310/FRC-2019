package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.subsystems.Drive.DriveControlMode;
import frc.team3310.robot.Robot;

public class DrivePathCameraTrack extends Command {

	public DrivePathCameraTrack() {
		requires(Robot.drive);
	}

	protected void initialize() {
		System.out.println("Start camera track");
		// Robot.drive.setLimeLED(true);
		Robot.drive.setCameraTrack();
		// setTimeout(timeout);
		Robot.drive.isLimeValid = true;
	}

	protected void execute() {
	}

	protected boolean isFinished() {
		return Robot.drive.isLimeValid == false;
	}

	protected void end() {
		System.out.println("camera track finished");
		System.out.println("Time to eject done camera track");
		Robot.drive.setControlMode(DriveControlMode.JOYSTICK);
	}

	protected void interrupted() {
		System.out.println("camera track interrupted");
		System.out.println("Time to eject done camera track");
		end();
	}
}