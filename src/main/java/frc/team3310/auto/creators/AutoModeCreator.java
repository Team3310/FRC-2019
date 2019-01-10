package frc.team3310.auto.creators;

import frc.team3310.auto.AutoModeBase;
import frc.team3310.robot.AutoFieldState;

public interface AutoModeCreator {
    AutoModeBase getStateDependentAutoMode(AutoFieldState fieldState);
}
