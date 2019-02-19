package frc.team3310.robot.commands;

import frc.team3310.robot.Robot;

/**
 *
 */
public class IntakeSetSpeedFrontSensorOff extends ExtraTimeoutCommand {
	
	private double speed;
	private boolean cubeDetected;
	private double EXTRA_INTAKE_TIME = 0.05;
	private static final double TIMEOUT = 10.0;

    public IntakeSetSpeedFrontSensorOff(double speed) {
    	this.speed = speed;
        requires(Robot.intake);
    }

    public IntakeSetSpeedFrontSensorOff(double speed, double extraTimeout) {
    	this.speed = speed;
    	EXTRA_INTAKE_TIME = extraTimeout;
        requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Intake sensor off started");
    	resetExtraOneTimer();
    	setTimeout(TIMEOUT);
		cubeDetected = false;
    	Robot.intake.setSpeed(speed);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (cubeDetected == false && (Robot.intake.getFrontRightIRIntakeSensor() || Robot.intake.getFrontVEXIntakeSensor())) {
    		startExtraOneTimeout(EXTRA_INTAKE_TIME);
    		cubeDetected = true;
    		System.out.println("CUBE DETECTED!!!!!");
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isExtraOneTimedOut() || isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.intake.setSpeed(0.03);
    	System.out.println("Intake sensor off end!!!!");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("IntakeSetSpeedFrontSensorOff interrupted");
    }
}
