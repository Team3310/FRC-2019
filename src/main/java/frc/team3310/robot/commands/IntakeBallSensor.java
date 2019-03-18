package frc.team3310.robot.commands;

import frc.team3310.robot.Robot;
import frc.team3310.robot.subsystems.Intake;
import frc.team3310.robot.subsystems.Intake.BallArmState;

/**
 *
 */
public class IntakeBallSensor extends ExtraTimeoutCommand {

    private double speed;
    private boolean ballDetected;
    private double EXTRA_INTAKE_TIME = 0.01;
    private double EXTRA_INTAKE_DEPLOY_TIME = .7;
    private static final double TIMEOUT = 10.0;

    public IntakeBallSensor(double speed) {
        this.speed = speed;
        requires(Robot.intake);
    }

    public IntakeBallSensor(double speed, double afterSensorTimeout) {
        this.speed = speed;
        EXTRA_INTAKE_TIME = afterSensorTimeout;
        requires(Robot.intake);
    }

    public IntakeBallSensor(double speed, double afterSensorTimeout, double deployIntakeTimeout) {
        this.speed = speed;
        EXTRA_INTAKE_TIME = afterSensorTimeout;
        EXTRA_INTAKE_DEPLOY_TIME = deployIntakeTimeout;
        requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        System.out.println("Intake sensor off started");
        resetExtraOneTimer();
        resetExtraTwoTimer();
        startExtraTwoTimeout(EXTRA_INTAKE_DEPLOY_TIME);
        setTimeout(TIMEOUT);
        ballDetected = false;
        Robot.intake.setSpeed(speed);
        Robot.intake.setBallArmState(BallArmState.OUT);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        if (ballDetected == false && Robot.intake.getFrontRightIRIntakeSensor()
                && Robot.intake.getFrontLeftIRIntakeSensor() && isExtraTwoTimedOut()) {
            startExtraOneTimeout(EXTRA_INTAKE_TIME);
            ballDetected = true;
            System.out.println("BALL DETECTED!!!!!");
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isExtraOneTimedOut() || isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.intake.setBallArmState(BallArmState.IN);
        Robot.intake.setSpeed(Intake.INTAKE_HOLD_SPEED);
        System.out.println("Intake sensor off end!!!!");
        Robot.elevator.elevatorCargoMode = true;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        System.out.println("IntakeSetSpeedFrontSensorOff interrupted");
        Robot.intake.setSpeed(0.0);
    }
}
