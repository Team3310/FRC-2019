package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.Robot;

/**
 *
 */
public class IntakeSetSpeedTimed extends Command {
	
	private double speed;
	private double timeout;

    public IntakeSetSpeedTimed(double speed, double timeout) {
    	this.speed = speed;
    	this.timeout = timeout;
        requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	setTimeout(timeout);
    	Robot.intake.setSpeed(speed);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.intake.setSpeed(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
