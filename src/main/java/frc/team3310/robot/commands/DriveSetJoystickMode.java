package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.Robot;
import frc.team3310.robot.subsystems.Drive.DriveControlMode;

public class DriveSetJoystickMode extends Command {

  public DriveSetJoystickMode() {
    requires(Robot.drive);
  }

  protected void initialize() {
    Robot.drive.setControlMode(DriveControlMode.JOYSTICK);
  }

  protected void execute() {
  }

  protected boolean isFinished() {
    return true;
  }

  protected void end() {

  }

  protected void interrupted() {
    end();
  }
}