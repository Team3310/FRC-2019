package frc.team3310.robot.commands;

import frc.team3310.utility.lib.control.RobotStatus;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class WaitUntilCrossXBoundaryPostiveCommand extends Command {

    private double mXBoundary = 0;

    public WaitUntilCrossXBoundaryPostiveCommand(double x) {
        mXBoundary = x;
    }

    @Override
    public boolean isFinished() {
        return RobotStatus.getInstance().getFieldToVehicle(Timer.getFPGATimestamp()).getTranslation().x() > mXBoundary;

    }

    @Override
    public void execute() {

    }

    @Override
    public void end() {
        System.out.println("Passes Boundary");
    }

    @Override
    public void initialize() {

    }
}
