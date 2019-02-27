package frc.team3310.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.utility.lib.physics.DriveCharacterization;

public class CalculateCharacterization extends Command {

    private List<DriveCharacterization.VelocityDataPoint> velocityData;
    private List<DriveCharacterization.AccelerationDataPoint> accelerationData;

    public CalculateCharacterization(List<DriveCharacterization.VelocityDataPoint> velocityData, List<DriveCharacterization.AccelerationDataPoint> accelerationData) {
        this.velocityData = velocityData;
        this.accelerationData = accelerationData;
    }

    @Override
    protected void initialize() {
        DriveCharacterization.CharacterizationConstants constants = DriveCharacterization.characterizeDrive(velocityData, accelerationData);

        System.out.println("ks: " + constants.ks);
        System.out.println("kv: " + constants.kv);
        System.out.println("ka: " + constants.ka);
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
