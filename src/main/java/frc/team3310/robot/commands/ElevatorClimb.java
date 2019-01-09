package frc.team3310.robot.commands;

import frc.team3310.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ElevatorClimb extends CommandGroup {

    public ElevatorClimb() {
        addSequential(new ElevatorSpeedShift(Elevator.ElevatorSpeedShiftState.LO));
        addSequential(new ElevatorSetSpeed(Elevator.CLIMB_SPEED));
    }
}
