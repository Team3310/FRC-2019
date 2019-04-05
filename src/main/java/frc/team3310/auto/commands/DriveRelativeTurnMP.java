package frc.team3310.auto.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.Robot;
import frc.team3310.robot.paths.TrajectoryGenerator.RightLeftAutonSide;
import frc.team3310.robot.subsystems.Drive.DriveControlMode;
import frc.team3310.utility.MPSoftwarePIDController.MPSoftwareTurnType;

public class DriveRelativeTurnMP extends Command {
	private double relativeTurnAngleDeg, maxTurnRateDegPerSec;
	private MPSoftwareTurnType turnType;

	public DriveRelativeTurnMP(double relativeTurnAngleDeg, double maxTurnRateDegPerSec, MPSoftwareTurnType turnType) {
		requires(Robot.drive);
		this.relativeTurnAngleDeg = relativeTurnAngleDeg;
		this.maxTurnRateDegPerSec = maxTurnRateDegPerSec;
		this.turnType = turnType;
	}

	protected void initialize() {
		if (Robot.trajectoryGenerator.getRightLeftAutonSide() == RightLeftAutonSide.RIGHT) {
			relativeTurnAngleDeg = -relativeTurnAngleDeg;
		}
		Robot.drive.setRelativeMaxTurnMP(relativeTurnAngleDeg, maxTurnRateDegPerSec, turnType);
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