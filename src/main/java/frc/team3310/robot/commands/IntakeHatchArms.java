package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.Robot;
import frc.team3310.robot.subsystems.Intake.HatchArmState;

public class IntakeHatchArms extends Command {
	private HatchArmState state;

	public IntakeHatchArms(HatchArmState state) {
		// requires(Robot.drive);
		this.state = state;
	}

	@Override
	protected void initialize() {
		Robot.intake.setHatchArmState(state);
		// System.out.println("Hatch Arms " + state);
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