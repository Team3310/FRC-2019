package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.Robot;
import frc.team3310.robot.subsystems.Drive.DriveSpeedShiftState;

public class DriveSpeedShift extends Command {
	private DriveSpeedShiftState state;
	
	public DriveSpeedShift(DriveSpeedShiftState state) {
//		requires(Robot.drive);
		this.state = state;
	}

	@Override
	protected void initialize() {
		//Robot.drive.setShiftState(state);
		//Robot.drive.configureTalonsForSpeedControl(); //TODO see if this is needed
		System.out.println("Shift " + state);
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