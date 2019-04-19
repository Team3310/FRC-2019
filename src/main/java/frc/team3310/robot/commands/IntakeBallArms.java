package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.Robot;
import frc.team3310.robot.subsystems.Intake.BallArmState;

public class IntakeBallArms extends Command {
	private BallArmState state;

	public IntakeBallArms(BallArmState state) {
		// requires(Robot.drive);
		this.state = state;
	}

	@Override
	protected void initialize() {
		// System.out.println("Start Eject ");
		Robot.intake.setBallArmState(state);
		// System.out.println("Ball Arms " + state);
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