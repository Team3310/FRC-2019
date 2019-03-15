package frc.team3310.robot.commands;

import frc.team3310.robot.Constants;
import frc.team3310.robot.Robot;
import frc.team3310.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.command.Command;

public class ElevatorAutoZero extends Command {
	private double MIN_ELEVATOR_POSITION_CHANGE = 0.05;
	private double lastElevatorPosition;
	private int encoderCount;

	public ElevatorAutoZero(boolean interrutible) {
		requires(Robot.elevator);
		setInterruptible(interrutible);
	}

	@Override
	protected void initialize() {
		lastElevatorPosition = Constants.MAX_POSITION_INCHES;
		Robot.elevator.setSpeed(Elevator.AUTO_ZERO_SPEED);
		encoderCount = 0;
		System.out.println("Auto zero initialize");
	}

	@Override
	protected void execute() {

	}

	@Override
	protected boolean isFinished() {
		double currentElevatorPosition = Robot.elevator.getElevatorPositionInches();
		double elevatorPositionChange = lastElevatorPosition - currentElevatorPosition;
		lastElevatorPosition = currentElevatorPosition;
		boolean test = encoderCount > 2 && Math.abs(elevatorPositionChange) < MIN_ELEVATOR_POSITION_CHANGE
				&& Robot.elevator.getAverageMotorCurrent() > Elevator.AUTO_ZERO_MOTOR_CURRENT;
		System.out.println("encoderCount = " + encoderCount + ", test = " + test + ", elevator change = "
				+ elevatorPositionChange + ", current = " + Robot.elevator.getAverageMotorCurrent());

		if (Math.abs(elevatorPositionChange) < MIN_ELEVATOR_POSITION_CHANGE) {
			encoderCount++;
		} else {
			encoderCount = 0;
		}

		return test;
	}

	@Override
	protected void end() {
		Robot.elevator.setSpeed(0);
		Robot.elevator.resetEncoders(Constants.LOW_HOME_POSITION_INCHES);
		Robot.elevator.setElevatorMotionMagicPosition(Constants.LOW_HOME_POSITION_INCHES);
		System.out.println("Elevator Zeroed");
	}

	@Override
	protected void interrupted() {

	}
}