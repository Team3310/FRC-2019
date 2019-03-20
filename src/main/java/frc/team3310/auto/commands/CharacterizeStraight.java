/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.auto.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.team3310.utility.lib.physics.DriveCharacterization;

public class CharacterizeStraight extends CommandGroup {

  public CharacterizeStraight() {
    ArrayList<DriveCharacterization.VelocityDataPoint> velocityData = new ArrayList<DriveCharacterization.VelocityDataPoint>();
    ArrayList<DriveCharacterization.AccelerationDataPoint> accelerationData = new ArrayList<DriveCharacterization.AccelerationDataPoint>();

    addSequential(new CollectVelocityData(velocityData, false, false));
    addSequential(new WaitCommand(10));
    addSequential(new CollectAccelerationData(accelerationData, false, false));
    addSequential(new WaitCommand(10));
    addSequential(new CalculateCharacterization(velocityData, accelerationData));
  }
}
