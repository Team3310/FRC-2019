package frc.team3310.auto.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team3310.utility.lib.trajectory.LazyLoadTrajectory;
import frc.team3310.utility.lib.trajectory.MirroredTrajectory;

public class LazyLoadCommandGroup extends CommandGroup {
    private ArrayList<LazyLoadTrajectory> trajectories = new ArrayList<LazyLoadTrajectory>();
    
    public MirroredTrajectory registerTrajectory(LazyLoadTrajectory trajectory) {
        trajectories.add(trajectory);
        return trajectory.getTrajectory();
    }

    public void activate() {
        for(LazyLoadTrajectory trajectory : trajectories) {
            trajectory.activate();
        }
    };

}
