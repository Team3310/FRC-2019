package frc.team3310.auto.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.subsystems.Drive.DriveControlMode;
import frc.team3310.utility.MPSoftwarePIDController.MPSoftwareTurnType;
import frc.team3310.robot.Robot;

public class DriveVelocityWithDistance extends Command {
	private double speed_inches_s, distance_inches;

	public DriveVelocityWithDistance(double speed_inches_s, double distance_inches) {
		requires(Robot.drive);
		this.speed_inches_s = speed_inches_s;
		this.distance_inches = distance_inches;
	}

	protected void initialize() {
		Robot.drive.resetEncoders();
		Robot.drive.setVelocitySetpoint(speed_inches_s, speed_inches_s);
	}

	protected void execute() {
	}

	protected boolean isFinished() {
		return Math.abs((Robot.drive.getLeftPositionInches() + Robot.drive.getRightPositionInches()) / 2) > Math
				.abs(distance_inches);
	}

	protected void end() {
		System.out.println("I'm Done");
		Robot.drive.setControlMode(DriveControlMode.JOYSTICK);
	}

	protected void interrupted() {
	}
}