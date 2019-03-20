package frc.team3310.auto.commands;

import frc.team3310.utility.lib.control.RobotStatus;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class WaitUntilCrossXBoundary extends Command {

    public enum MovingXDirection {Negative, Positive};
 
    private double mXBoundary = 0;
    private MovingXDirection mMovingDirection;

    public WaitUntilCrossXBoundary(double x, MovingXDirection movingDirecton) {
        mXBoundary = x;
        mMovingDirection = movingDirecton;
    }

    @Override
    public boolean isFinished() {
        System.out.println("X Position" +RobotStatus.getInstance().getFieldToVehicle(Timer.getFPGATimestamp()).getTranslation().x() + ", Y Position" +RobotStatus.getInstance().getFieldToVehicle(Timer.getFPGATimestamp()).getTranslation().y());
        if (mMovingDirection == MovingXDirection.Positive) {
            return RobotStatus.getInstance().getFieldToVehicle(Timer.getFPGATimestamp()).getTranslation().x() > mXBoundary;
        }
        else {
            return RobotStatus.getInstance().getFieldToVehicle(Timer.getFPGATimestamp()).getTranslation().x() < mXBoundary;
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public void end() {
        System.out.println("Passed X Boundary");
    }

    @Override
    public void initialize() {

    }
}
