package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team3310.robot.subsystems.Elevator;
import frc.team3310.robot.subsystems.Intake;

/**
 *
 */
public class IntakeCubeAndLift extends CommandGroup {

    public IntakeCubeAndLift() {
        addSequential(new ElevatorSetPositionMP(Elevator.MIN_POSITION_INCHES));
//        addSequential(new ElevatorAutoZero(true));
        addSequential(new IntakeSetSpeedFrontSensorOff(Intake.INTAKE_LOAD_SPEED));
//        addSequential(new ElevatorSetPositionMP(Elevator.AFTER_INTAKE_POSITION_INCHES));
//        addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_LOAD_SLOW_SPEED, 0.2));
//      addSequential(new IntakeSetSpeed(0.05));
    }
}
