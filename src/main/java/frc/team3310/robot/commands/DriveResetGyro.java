package frc.team3310.robot.commands;

import frc.team3310.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveResetGyro extends Command
{
	public DriveResetGyro() {
		requires(Robot.drive);
	}

	@Override
	protected void initialize() {
		Robot.drive.resetGyro();
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