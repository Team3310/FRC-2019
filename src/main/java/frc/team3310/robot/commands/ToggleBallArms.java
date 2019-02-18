package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.Robot;
import frc.team3310.robot.subsystems.Intake.BallArmState;

public class ToggleBallArms extends Command {
	public ToggleBallArms() {
		// requires(Robot.drive);
	}

	@Override
	protected void initialize() {
		System.out.println("Ball Arms Out");
		Robot.intake.setBallArmState(BallArmState.OUT);
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
		System.out.println("Hatch Arms In");
		Robot.intake.setBallArmState(BallArmState.IN);

	}

	@Override
	protected void interrupted() {

	}
}