/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team3310.robot.Constants;
import frc.team3310.robot.paths.TrajectoryGenerator;
import frc.team3310.robot.subsystems.Intake.HatchArmState;
import frc.team3310.utility.MPSoftwarePIDController.MPSoftwareTurnType;

public class AutoStartLevel1OutsideRocketFront extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutoStartLevel1OutsideRocketFront() {
    addParallel(new ElevatorSetPositionMM(Constants.HATCH_LEVEL_1));
    addSequential(new DriveMotionCommand(
        TrajectoryGenerator.getInstance().getTrajectorySet().level1StartToRocketFront.right, true));
    addSequential(new DriveSetSpeed(0.3, 0.2));
    addSequential(new EjectHatch(), .8);
    addParallel(new IntakeHatch());
    // addSequential(
    // new
    // DriveMotionCommand(TrajectoryGenerator.getInstance().getTrajectorySet().rocketFrontToLoading.right,
    // false));
    // addSequential(new DriveAbsoluteTurnMP(180, 180, MPSoftwareTurnType.TANK));
    // addSequential(new DriveSetSpeed(0.3, 0.5));
    // addSequential(new IntakeHatchArms(HatchArmState.IN));
    // addSequential(
    // new
    // DriveMotionCommand(TrajectoryGenerator.getInstance().getTrajectorySet().loadingToRocketBack.right,
    // false));
  }
}
