package frc.team3310.robot.commands;

import frc.team3310.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ElevatorSetPositionMP extends Command {
	
	private double targetPositionInches;
	private boolean isAtTarget;
	private static final double MIN_DELTA_TARGET = 0.3;

    public ElevatorSetPositionMP(double targetPositionInches) {
    	this.targetPositionInches = targetPositionInches;
        requires(Robot.elevator);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if (Math.abs(targetPositionInches - Robot.elevator.getPositionInches()) < MIN_DELTA_TARGET) {
    		isAtTarget = true;
    	}
    	else {
        	isAtTarget = false;
        	Robot.elevator.setPositionMP(targetPositionInches);
    	}
//    	System.out.println("Elevator set MP initialized, target = " + targetPositionInches);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isAtTarget || Robot.elevator.isFinished();
    }

    // Called once after isFinished returns true
    protected void end() {
		Robot.elevator.setPositionPID(Robot.elevator.getPositionInches());
//    	System.out.println("Elevator set MP end");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
//    	System.out.println("ElevatorSetPositionMP interrupted");
    	Robot.elevator.setFinished(true);
		Robot.elevator.setPositionPID(Robot.elevator.getPositionInches());
    }
}
