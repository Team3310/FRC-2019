package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.Robot;
import frc.team3310.robot.subsystems.Climb.ForkShiftState;

public class ToggleForks extends Command {
  private ForkShiftState state;

  public ToggleForks(ForkShiftState state) {
    this.state = state;
  }

  @Override
  protected void initialize() {
    Robot.climb.setForkState(state);
    // System.out.println("Fork Arms " + state);
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