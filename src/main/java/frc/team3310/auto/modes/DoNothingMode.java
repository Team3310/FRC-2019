package frc.team3310.auto.modes;

import frc.team3310.auto.AutoModeBase;
import frc.team3310.auto.AutoModeEndedException;

public class DoNothingMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Doing nothing");
    }
}
