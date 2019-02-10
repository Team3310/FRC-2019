package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.Robot;
import frc.team3310.robot.subsystems.Elevator.ElevatorClimbShiftState;

public class ElevatorClimbShift extends Command {
  private ElevatorClimbShiftState state;

  public ElevatorClimbShift(ElevatorClimbShiftState state) {
    requires(Robot.elevator);
    this.state = state;
  }

  @Override
  protected void initialize() {
    Robot.elevator.setElevatorClimbState(state);
    System.out.println("Shift " + state);
  }

  @Override
  protected void execute() {

  }

  @Override
  protected boolean isFinished() {
    return true;
  }

  @Override
  protected void end() {

  }

  @Override
  protected void interrupted() {

  }
}