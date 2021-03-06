package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.Robot;
import frc.team3310.robot.subsystems.Elevator.ElevatorControlMode;

/**
 *
 */
public class ElevatorSetMode extends Command {

	private ElevatorControlMode controlMode;
	
    public ElevatorSetMode(ElevatorControlMode controlMode) {
    	this.controlMode = controlMode;
        requires(Robot.elevator);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if (controlMode == ElevatorControlMode.JOYSTICK_POSITION_PID) {
    		Robot.elevator.setJoystickPID();
            // System.out.println("In PID joystick mode");
    	}
    	else if (controlMode == ElevatorControlMode.JOYSTICK_MANUAL) {
            Robot.elevator.setJoystickOpenLoop();
            // System.out.println("In manual joystick mode");
    	}
    	else {
    		Robot.elevator.setSpeed(0.0);
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
