package frc.team3310.auto.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.Robot;
import frc.team3310.robot.paths.TrajectoryGenerator.RightLeftAutonSide;
import frc.team3310.utility.lib.control.RobotStatus;

public class WaitUntilCrossYBoundary extends Command {

    public enum MovingYDirection {
        OutsideToInside, InsideToOutside
    };

    private double mYBoundary = 0;
    private double mFlip = 1;
    private MovingYDirection mMovingDirection;
    private boolean validDirection = false;

    public WaitUntilCrossYBoundary(double yForRightSideAuton, MovingYDirection movingDirection) {
        mYBoundary = yForRightSideAuton;
        mMovingDirection = movingDirection;
    }

    @Override
    public void initialize() {
        RightLeftAutonSide autonSide = Robot.trajectoryGenerator.getRightLeftAutonSide();
        if (autonSide == RightLeftAutonSide.LEFT) {
            mFlip = -1;
        }
        validDirection = false;
    }

    @Override
    public boolean isFinished() {
        if (validDirection) {
            if (mMovingDirection == MovingYDirection.OutsideToInside) {
                return mFlip * RobotStatus.getInstance().getFieldToVehicle().getTranslation().y() > mYBoundary;
            } else {
                return mFlip * RobotStatus.getInstance().getFieldToVehicle().getTranslation().y() < mYBoundary;
            }
        } else {
            if (mMovingDirection == MovingYDirection.OutsideToInside) {
                validDirection = mFlip
                        * RobotStatus.getInstance().getFieldToVehicle().getTranslation().y() < mYBoundary;
            } else {
                validDirection = mFlip
                        * RobotStatus.getInstance().getFieldToVehicle().getTranslation().y() > mYBoundary;
            }
            if (validDirection == true) {
                // System.out.println(
                //         "validDirection y = " + RobotStatus.getInstance().getFieldToVehicle().getTranslation().y());
            }
            return false;
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public void end() {
        // System.out
        //         .println("Passed Y Boundary Y = " + RobotStatus.getInstance().getFieldToVehicle().getTranslation().y());
    }
}
