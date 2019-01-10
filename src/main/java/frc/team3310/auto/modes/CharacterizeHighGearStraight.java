package frc.team3310.auto.modes;

import java.util.ArrayList;
import java.util.List;

import frc.team3310.auto.AutoModeBase;
import frc.team3310.auto.AutoModeEndedException;
import frc.team3310.auto.actions.CollectAccelerationData;
import frc.team3310.auto.actions.CollectVelocityData;
import frc.team3310.auto.actions.WaitAction;
import frc.team3310.utility.lib.physics.DriveCharacterization;

public class CharacterizeHighGearStraight extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        List<DriveCharacterization.VelocityDataPoint> velocityData = new ArrayList<>();
        List<DriveCharacterization.AccelerationDataPoint> accelerationData = new ArrayList<>();

        // runAction(new ShiftHighGearAction(false));
        // runAction(new WaitAction(10));

        runAction(new CollectVelocityData(velocityData, false, false, true));
        runAction(new WaitAction(10));
        runAction(new CollectAccelerationData(accelerationData, false, false, true));

        DriveCharacterization.CharacterizationConstants constants = DriveCharacterization.characterizeDrive(velocityData, accelerationData);

        System.out.println("ks: " + constants.ks);
        System.out.println("kv: " + constants.kv);
        System.out.println("ka: " + constants.ka);
    }
}
