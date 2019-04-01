package frc.team3310.auto.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.subsystems.Drive.DriveControlMode;
import frc.team3310.utility.MPSoftwarePIDController.MPSoftwareTurnType;
import frc.team3310.robot.Robot;

public class DriveAbsoluteTurnMP extends Command {
	private double absoluteTurnAngleDeg, maxTurnRateDegPerSec;
	private MPSoftwareTurnType turnType;

	public DriveAbsoluteTurnMP(double absoluteTurnAngleDeg, double maxTurnRateDegPerSec, MPSoftwareTurnType turnType) {
		requires(Robot.drive);
		this.absoluteTurnAngleDeg = absoluteTurnAngleDeg;
		this.maxTurnRateDegPerSec = maxTurnRateDegPerSec;
		this.turnType = turnType;
	}

	protected void initialize() {
		// if (Robot.drive.isRed() == false) {
		// absoluteTurnAngleDeg = absoluteTurnAngleDeg * -1;
		// }
		Robot.drive.overrideTrajectory(true);
		Robot.drive.setAbsoluteMaxTurnMP(absoluteTurnAngleDeg, maxTurnRateDegPerSec, turnType);
	}

	protected void execute() {
	}

	protected boolean isFinished() {
		return Robot.drive.isFinished();
	}

	protected void end() {
		Robot.drive.setControlMode(DriveControlMode.JOYSTICK);
	}

	protected void interrupted() {
		end();
	}
}