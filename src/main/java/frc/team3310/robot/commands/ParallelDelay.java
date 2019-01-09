package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class ParallelDelay extends CommandGroup {

    public ParallelDelay(double timeout, Command command) {
    	addSequential(new WaitCommand(timeout));
        addSequential(command);
    }
}
