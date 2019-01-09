package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.team3310.robot.subsystems.Elevator;

/**
 *
 */
public class IntakeDropAndHold extends CommandGroup {

    public IntakeDropAndHold(double speed, double timeout) {
    	addSequential(new IntakeSetSpeedTimed(speed, timeout));
       	addSequential(new WaitCommand(1.0));
        addSequential(new ElevatorSetPositionMP(Elevator.SWITCH_POSITION_INCHES));
    }
}
