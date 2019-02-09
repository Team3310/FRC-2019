package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.Robot;
import frc.team3310.robot.subsystems.Intake.HatchArmState;

public class ToggleHatchArms extends Command {
	private HatchArmState state;
	
	public ToggleHatchArms() {
//		requires(Robot.drive);
	}

	@Override
	protected void initialize() {
		System.out.println("Hatch Arms Out");
		Robot.intake.setHatchArmState(HatchArmState.OUT);
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
		Robot.intake.setHatchArmState(HatchArmState.IN);

	}

	@Override
	protected void interrupted() {
			
	}
}