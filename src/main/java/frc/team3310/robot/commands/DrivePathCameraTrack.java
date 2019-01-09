package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.subsystems.Drive.DriveControlMode;
import frc.team3310.robot.Robot;

public class DrivePathCameraTrack extends Command {
	private double straightVelocity;
	private double timeout;

	public DrivePathCameraTrack(double straightVelocity, double timeout) {
        this.straightVelocity = straightVelocity;
        this.timeout = timeout;
		requires(Robot.drive);
	}

	protected void initialize() {
		System.out.println("Start camera track");
		Robot.drive.setLimeLED(true);
		//Robot.drive.setCameraTrack(straightVelocity); //TODO Add setCameraTrack Back
		setTimeout(timeout);
	}

	protected void execute() {
	}

	protected boolean isFinished() {
		return isTimedOut() || Robot.drive.isFinished(); 
	}

	protected void end() {
		System.out.println("camera track finished");
		Robot.drive.setLimeLED(false);
		Robot.drive.setControlMode(DriveControlMode.JOYSTICK);
	}

	protected void interrupted() {
    	System.out.println("camera track interrupted");
		end();
	}
}