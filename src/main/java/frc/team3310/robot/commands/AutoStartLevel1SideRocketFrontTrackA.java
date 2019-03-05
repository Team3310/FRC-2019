/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.team3310.robot.paths.TrajectoryGenerator;
import frc.team3310.robot.subsystems.Intake.HatchArmState;
import frc.team3310.robot.Constants;

public class AutoStartLevel1SideRocketFrontTrackA extends CommandGroup {
    /**
     * Add your docs here.
     */
    public AutoStartLevel1SideRocketFrontTrackA() {
        addParallel(new ElevatorSetPositionMM(Constants.HATCH_LEVEL_1));

        addParallel(new AutoCameraTrackWhenCrossedBoundary(165));
        addSequential(new DriveMotionCommand(TrajectoryGenerator.getInstance().getTrajectorySet().level1StartToRocketFront, true));
        addSequential(new EjectHatch());
        addSequential(new WaitCommand("Eject Break", 1));
        addParallel(new IntakeHatch());
        addSequential(new DriveMotionCommand(TrajectoryGenerator.getInstance().getTrajectorySet().rocketFrontToTurn1A, false));

        addParallel(new AutoCameraTrackWhenCrossXBoundaryNegative(45));
        addParallel(new DriveMotionCommand(TrajectoryGenerator.getInstance().getTrajectorySet().rocketFrontTurn1AToLoading, false), 4);
        addSequential(new IntakeHatchArms(HatchArmState.IN));
        addSequential(new DriveMotionCommand(TrajectoryGenerator.getInstance().getTrajectorySet().loadingToRocketBack, false));
        addParallel(new AutoCameraTrackWhenCrossXBoundaryNegative(25)); // 25
        addSequential(new DriveMotionCommand(TrajectoryGenerator.getInstance().getTrajectorySet().turn3ToRocketBack, false));
    }
}
