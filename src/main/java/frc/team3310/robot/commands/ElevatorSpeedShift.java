package frc.team3310.robot.commands;

import frc.team3310.robot.Robot;
import frc.team3310.robot.subsystems.Elevator.ElevatorSpeedShiftState;

import edu.wpi.first.wpilibj.command.Command;

public class ElevatorSpeedShift extends Command
{
	private ElevatorSpeedShiftState state;
	
	public ElevatorSpeedShift(ElevatorSpeedShiftState state) {
		requires(Robot.elevator);
		this.state = state;
	}

	@Override
	protected void initialize() {
		Robot.elevator.setShiftState(state);
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