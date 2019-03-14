/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3310.robot.commands.AutoStartLevel1SideCargoFront2;
import frc.team3310.robot.commands.AutoStartLevel1SideCargoFrontSide1;
import frc.team3310.robot.commands.AutoStartLevel1SideRocketFrontBackLow;
import frc.team3310.robot.commands.ElevatorAutoZero;
import frc.team3310.robot.loops.Looper;
import frc.team3310.robot.paths.TrajectoryGenerator;
import frc.team3310.robot.paths.TrajectoryGenerator.RightLeftAutonSide;
import frc.team3310.robot.subsystems.AirCompressor;
import frc.team3310.robot.subsystems.Drive;
import frc.team3310.robot.subsystems.Drive.DriveControlMode;
import frc.team3310.robot.subsystems.Elevator;
import frc.team3310.robot.subsystems.Intake;
import frc.team3310.robot.subsystems.RobotStateEstimator;
import frc.team3310.utility.lib.control.RobotStatus;

public class Robot extends TimedRobot {
	public static OI oi;

	// Declare subsystems
	public static final Drive drive = Drive.getInstance();
	public static final Elevator elevator = Elevator.getInstance();
	public static final Intake intake = Intake.getInstance();
	public static final AirCompressor compressor = AirCompressor.getInstance();
	public static final RobotStateEstimator estimator = RobotStateEstimator.getInstance();
	public static final TrajectoryGenerator trajectoryGenerator = TrajectoryGenerator.getInstance();

	// Control looper
	public static final Looper controlLoop = new Looper();

	// Choosers
	private SendableChooser<OperationMode> operationModeChooser;
	private SendableChooser<Command> autonTaskChooser;
	private SendableChooser<RightLeftAutonSide> autonRightLeftChooser;
	private Command autonomousCommand;

	public static enum OperationMode {
		TEST, PRACTICE, COMPETITION
	};

	public static OperationMode operationMode = OperationMode.COMPETITION;

	public static RightLeftAutonSide rightLeftSide = RightLeftAutonSide.RIGHT;

	// PDP
	public static final PowerDistributionPanel pdp = new PowerDistributionPanel();

	// State
	private RobotStatus robotState = RobotStatus.getInstance();

	public Robot() {
		super(Constants.kLooperDt * 2);
		System.out.println("Main loop period = " + getPeriod());
	}

	public void zeroAllSensors() {
		drive.zeroSensors();
		elevator.resetEncoders();
	}

	// Called at the start of connection
	@Override
	public void robotInit() {
		oi = OI.getInstance();

		controlLoop.register(drive);
		controlLoop.register(elevator);
		RobotStateEstimator.getInstance().registerEnabledLoops(controlLoop);
		trajectoryGenerator.generateTrajectories();

		operationModeChooser = new SendableChooser<OperationMode>();
		operationModeChooser.addOption("Practice", OperationMode.PRACTICE);
		operationModeChooser.setDefaultOption("Competition", OperationMode.COMPETITION);
		operationModeChooser.addOption("Test", OperationMode.TEST);
		SmartDashboard.putData("Operation Mode", operationModeChooser);

		autonTaskChooser = new SendableChooser<Command>();
		autonTaskChooser.setDefaultOption("L1 Rocket Front/Back Low", new AutoStartLevel1SideRocketFrontBackLow());

		autonTaskChooser.addOption("L1 Cargo Front/Side", new AutoStartLevel1SideCargoFrontSide1());

		autonTaskChooser.addOption("L1 Cargo Front/Front", new AutoStartLevel1SideCargoFront2());

		SmartDashboard.putData("Autonomous", autonTaskChooser);

		autonRightLeftChooser = new SendableChooser<RightLeftAutonSide>();
		autonRightLeftChooser.addOption("Left", RightLeftAutonSide.LEFT);
		autonRightLeftChooser.setDefaultOption("Right", RightLeftAutonSide.RIGHT);
		SmartDashboard.putData("Auton Side", autonRightLeftChooser);

		LiveWindow.setEnabled(false);
		LiveWindow.disableAllTelemetry();

		zeroAllSensors();
		compressor.turnCompressorOff();
		drive.setPipeline(1);
		Robot.elevator.setRobotScoreMode();
	}

	// Called every loop for all modes
	public void robotPeriodic() {
		updateStatus();
	}

	// Called once when is disabled
	@Override
	public void disabledInit() {
	}

	// Called constantly when the robot is disabled
	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		drive.setPipeline(1);
	}

	// Called once at the start of auto
	@Override
	public void autonomousInit() {
		controlLoop.start();
		drive.setIsRed(getAlliance().equals(Alliance.Red));
		drive.setPipeline(1);
		zeroAllSensors();

		rightLeftSide = autonRightLeftChooser.getSelected();
		trajectoryGenerator.setRightLeftAutonSide(rightLeftSide);

		autonomousCommand = autonTaskChooser.getSelected();

		if (autonomousCommand != null) {
			autonomousCommand.start();
		}
	}

	// Called constantly through auton
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	// Called once at the start of teleOp
	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();

		}

		operationMode = operationModeChooser.getSelected();
		drive.setControlMode(DriveControlMode.JOYSTICK);

		controlLoop.start();
		drive.setPipeline(1);
		drive.endGyroCalibration();
		Robot.elevator.setRobotScoreMode();

		if (operationMode != OperationMode.COMPETITION) {
			Scheduler.getInstance().add(new ElevatorAutoZero(true));
		}
	}

	// Called constantly through teleOp
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	public Alliance getAlliance() {
		return m_ds.getAlliance();
	}

	public double getMatchTime() {
		return m_ds.getMatchTime();
	}

	public void updateStatus() {
		drive.updateStatus(operationMode);
		elevator.updateStatus(operationMode);
		intake.updateStatus(operationMode);
		robotState.updateStatus(operationMode);
	}
}
