package frc.team3310.utility.lib.trajectory;

import frc.team3310.utility.lib.geometry.Pose2d;
import frc.team3310.utility.lib.geometry.Twist2d;

public interface IPathFollower {
    public Twist2d steer(Pose2d current_pose);

    public boolean isDone();
}
