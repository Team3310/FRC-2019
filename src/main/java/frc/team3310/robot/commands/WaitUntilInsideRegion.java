package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.Robot;
import frc.team3310.robot.paths.TrajectoryGenerator.RightLeftAutonSide;
import frc.team3310.utility.lib.control.RobotStatus;
import frc.team3310.utility.lib.geometry.Translation2d;

public class WaitUntilInsideRegion extends Command {
    private final static RobotStatus mRobotState = RobotStatus.getInstance();

    private Translation2d mBottomLeft;
    private Translation2d mTopRight;

    // (100, 100) (200, 200)
    public WaitUntilInsideRegion(Translation2d bottomLeftForRightSideAuton, Translation2d topRightForRightSideAuton) {
        mBottomLeft = bottomLeftForRightSideAuton;
        mTopRight = topRightForRightSideAuton;
    }

    @Override
    public void initialize() {
        RightLeftAutonSide autonSide = Robot.trajectoryGenerator.getRightLeftAutonSide();
        if (autonSide == RightLeftAutonSide.LEFT) {
            Translation2d bottomLeft = mBottomLeft;
            Translation2d topRight = mTopRight;
            mBottomLeft = new Translation2d(bottomLeft.x(), -topRight.y());
            mTopRight = new Translation2d(topRight.x(), -bottomLeft.y());
        } 
    }

    @Override
    public boolean isFinished() {
        Translation2d position = mRobotState.getLatestFieldToVehicle().getValue().getTranslation();
        return position.x() > mBottomLeft.x() && position.x() < mTopRight.x() && position.y() > mBottomLeft.y()
                && position.y() < mTopRight.y();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end() {

    }
}
