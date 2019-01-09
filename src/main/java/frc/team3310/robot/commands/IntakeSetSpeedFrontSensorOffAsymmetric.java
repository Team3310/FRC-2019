package frc.team3310.robot.commands;

import frc.team3310.robot.Robot;

/**
 *
 */
public class IntakeSetSpeedFrontSensorOffAsymmetric extends ExtraTimeoutCommand {
	
	private double speedLeft;
	private double speedRight;
	private boolean cubeDetected;
	private double EXTRA_INTAKE_TIME = 0.05;
	private static final double TIMEOUT = 10.0;

    public IntakeSetSpeedFrontSensorOffAsymmetric(double speedLeft, double speedRight) {
    	this.speedLeft = speedLeft;
    	this.speedRight = speedRight;
        requires(Robot.intake);
    }

    public IntakeSetSpeedFrontSensorOffAsymmetric(double speedLeft, double speedRight, double extraTimeout) {
    	this.speedLeft = speedLeft;
    	this.speedRight = speedRight;
    	EXTRA_INTAKE_TIME = extraTimeout;
        requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
//    	System.out.println("Intake sensor off started");
    	resetExtraTimer();
    	setTimeout(TIMEOUT);
		cubeDetected = false;
    	Robot.intake.setSpeedAsymmetric(speedLeft, speedRight);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (cubeDetected == false && (Robot.intake.getFrontIRIntakeSensor() || Robot.intake.getFrontLeftVEXIntakeSensor() || Robot.intake.getFrontRightVEXIntakeSensor())) {
    		startExtraTimeout(EXTRA_INTAKE_TIME);
    		cubeDetected = true;
//    		System.out.println("CUBE DETECTED!!!!!");
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isExtraTimedOut() || isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.intake.setSpeed(0);
//    	System.out.println("Intake sensor off end EJECT!!!!");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
//    	System.out.println("IntakeSetSpeedFrontSensorOff interrupted");
    }
}
