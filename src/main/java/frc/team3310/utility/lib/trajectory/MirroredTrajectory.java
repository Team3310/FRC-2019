package frc.team3310.utility.lib.trajectory;

import frc.team3310.utility.lib.geometry.Pose2dWithCurvature;
import frc.team3310.utility.lib.trajectory.timing.TimedState;

public class MirroredTrajectory {
    public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> right) {
            this.right = right;
            this.left = TrajectoryUtil.mirrorTimed(right);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
            return left ? this.left : this.right;
    }

    public final Trajectory<TimedState<Pose2dWithCurvature>> left;
    public final Trajectory<TimedState<Pose2dWithCurvature>> right;
}