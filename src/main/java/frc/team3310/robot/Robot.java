/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3310.robot.commands.ElevatorAutoZero;
import frc.team3310.robot.loops.Looper;
import frc.team3310.robot.subsystems.Drive;
import frc.team3310.robot.subsystems.Drive.DriveSpeedShiftState;
import frc.team3310.utility.lib.control.RobotStatus;
import frc.team3310.robot.subsystems.Elevator;
import frc.team3310.robot.subsystems.Intake;
import frc.team3310.robot.subsystems.RobotStateEstimator;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static OI oi;
	
	// Declare subsystems
	public static final Drive drive = Drive.getInstance();
	public static final Elevator elevator = Elevator.getInstance();
	public static final Intake intake = Intake.getInstance();
	
	// Control looper
	public static final Looper controlLoop = new Looper();
	
	// Choosers
	private SendableChooser<OperationMode> operationModeChooser;
	private SendableChooser<AutonRouteChooser> autonTaskChooser;
    private Command autonomousCommand;

	public static enum OperationMode { TEST, PRACTICE, COMPETITION };
	public static OperationMode operationMode = OperationMode.COMPETITION;

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
    }

	@Override
	public void robotInit() {
		
		oi = OI.getInstance();
		
    	controlLoop.register(drive);
    	controlLoop.register(elevator);
		RobotStateEstimator.getInstance().registerEnabledLoops(controlLoop);
 
    	// Update default at competition!!!
    	operationModeChooser = new SendableChooser<OperationMode>();
	    operationModeChooser.addOption("Practice", OperationMode.PRACTICE);
	    operationModeChooser.setDefaultOption("Competition", OperationMode.COMPETITION);
	    operationModeChooser.addOption("Test", OperationMode.TEST);
		SmartDashboard.putData("Operation Mode", operationModeChooser);

		autonTaskChooser = new SendableChooser<AutonRouteChooser>();


		SmartDashboard.putData("Auton Tasks", autonTaskChooser);
		
		LiveWindow.setEnabled(false);
		LiveWindow.disableAllTelemetry();

		zeroAllSensors();

		drive.setLimeLED(false);
		drive.setLimeCameraMode(false);
	}  
	
	// Called every loop for all modes
	public void robotPeriodic() {
		updateStatus();
	}

	@Override
	public void disabledInit() {
		drive.setLimeLED(false);
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
        zeroAllSensors();
	}

	@Override
	public void autonomousInit() {		
    	controlLoop.start();
    	drive.setIsRed(getAlliance().equals(Alliance.Red));
    	drive.setShiftState(DriveSpeedShiftState.LO);
    	elevator.setShiftState(Elevator.ElevatorSpeedShiftState.HI);
    	elevator.resetZeroPosition(Elevator.ZERO_POSITION_INCHES);
        zeroAllSensors();

		drive.setLimeLED(false);
		drive.setLimeCameraMode(false);
		
		String gameMessage = m_ds.getGameSpecificMessage();
		while (gameMessage == null || gameMessage.length() < 3) {
			gameMessage = m_ds.getGameSpecificMessage();
		}

		AutonRouteChooser routeChooser = autonTaskChooser.getSelected();
		autonomousCommand = routeChooser.getRoute(gameMessage);

		if (autonomousCommand != null) {
			autonomousCommand.start();
		}
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}

		operationMode = operationModeChooser.getSelected();
		
        controlLoop.start();
    	drive.setShiftState(Drive.DriveSpeedShiftState.LO);
    	drive.endGyroCalibration();
    	elevator.setShiftState(Elevator.ElevatorSpeedShiftState.HI);//
        zeroAllSensors();

		drive.setLimeLED(false);
		drive.setLimeCameraMode(false);
    	
    	if (operationMode != OperationMode.COMPETITION) {
    		Scheduler.getInstance().add(new ElevatorAutoZero(false));
    	}
    	else {
            elevator.setPositionPID(elevator.getPositionInches());
    	}
	}

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
    
    private class AutonRouteChooser {
    	
    	private HashMap<String, Command> commandMap = new HashMap<String, Command>(4);
    	
    	public AutonRouteChooser() {
    		
    	}
    	
    	public Command getRoute(String gameMessage) {
    		if (!commandMap.containsKey(gameMessage.toUpperCase())) {
    			System.out.println("Invalid game message");
    		}
    		return commandMap.get(gameMessage.toUpperCase());
    	}

    }

}
