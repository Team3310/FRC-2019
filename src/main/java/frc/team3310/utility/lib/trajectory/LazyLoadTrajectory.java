package frc.team3310.utility.lib.trajectory;

import frc.team3310.utility.lib.geometry.Pose2dWithCurvature;
import frc.team3310.utility.lib.trajectory.timing.TimedState;

public class LazyLoadTrajectory {

    public interface TrajectoryActivator {
        Trajectory<TimedState<Pose2dWithCurvature>> activateFunction();
    }

    private MirroredTrajectory trajectory;
    private TrajectoryActivator trajectoryActivator;

    public LazyLoadTrajectory(TrajectoryActivator trajectoryActivate) {
         this.trajectoryActivator = trajectoryActivate;
    }

    public MirroredTrajectory getTrajectory() {
        return trajectory;
    }

    public void activate() {
        if (trajectory == null) {
            trajectory = new MirroredTrajectory(trajectoryActivator.activateFunction());
        }
    }
}
