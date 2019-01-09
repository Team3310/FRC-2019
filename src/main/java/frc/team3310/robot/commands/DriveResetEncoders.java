package frc.team3310.robot.commands;

import frc.team3310.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveResetEncoders extends Command
{
	public DriveResetEncoders() {
		requires(Robot.drive);
	}

	@Override
	protected void initialize() {
		Robot.drive.resetEncoders();
	}

	@Override
	protected void execute() {
		
	}

	@Override
	protected boolean isFinished() {
		return true;
	}

	@Override
	protected void end() {
		
	}

	@Override
	protected void interrupted() {
			
	}
}