package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.utility.lib.control.RobotStatus;
import frc.team3310.utility.lib.geometry.Translation2d;

public class WaitUntilInsideRegion extends Command {
    private final static RobotStatus mRobotState = RobotStatus.getInstance();

    private final Translation2d mBottomLeft;
    private final Translation2d mTopRight;

    // (100, 100) (200, 200)
    public WaitUntilInsideRegion(Translation2d bottomLeft, Translation2d topRight, boolean isOnLeft) {
        if (isOnLeft) {
            mBottomLeft = new Translation2d(bottomLeft.x(), -topRight.y());
            mTopRight = new Translation2d(topRight.x(), -bottomLeft.y());
        } else {
            mBottomLeft = bottomLeft;
            mTopRight = topRight;
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

    @Override
    public void initialize() {

    }
}
