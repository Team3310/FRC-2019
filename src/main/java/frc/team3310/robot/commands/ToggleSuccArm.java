package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.Robot;
import frc.team3310.robot.subsystems.Climb.ArmShiftState;

public class ToggleSuccArm extends Command {
  private ArmShiftState state;

  public ToggleSuccArm(ArmShiftState state) {
    this.state = state;
  }

  @Override
  protected void initialize() {
    Robot.climb.setSuccArmState(state);
    System.out.println("Succ Arms " + state);
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