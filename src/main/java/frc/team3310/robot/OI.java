/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.InternalButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3310.robot.commands.DrivePathCameraTrack;
import frc.team3310.robot.commands.DrivePathCameraTrackStop;
import frc.team3310.robot.commands.DriveSpeedShift;
import frc.team3310.robot.commands.ElevatorSetMode;
import frc.team3310.robot.commands.ElevatorSetZero;
import frc.team3310.robot.commands.ElevatorSpeedShift;
import frc.team3310.robot.commands.IntakeBallArms;
import frc.team3310.robot.commands.IntakeHatchArms;
import frc.team3310.robot.commands.IntakeSetSpeed;
import frc.team3310.robot.commands.TurnCompressorOff;
import frc.team3310.robot.controller.GameController;
import frc.team3310.robot.controller.Playstation;
import frc.team3310.robot.controller.Xbox;
import frc.team3310.robot.subsystems.Drive;
import frc.team3310.robot.subsystems.Elevator;
import frc.team3310.robot.subsystems.Elevator.ElevatorControlMode;
import frc.team3310.robot.subsystems.Elevator.ElevatorSpeedShiftState;
import frc.team3310.robot.subsystems.Intake;
import frc.team3310.robot.subsystems.Intake.BallArmState;
import frc.team3310.robot.subsystems.Intake.HatchArmState;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  private static OI instance;
	
	
	private GameController m_driver;
	private GameController m_operator;


    public static OI getInstance() {
		if(instance == null) {
			instance = new OI();
		}
		return instance;
	}

	private OI() {
		// Driver controller
		m_driver = new GameController(RobotMap.DRIVER_JOYSTICK_1_USB_ID, new Xbox());
		m_operator = new GameController(RobotMap.OPERATOR_JOYSTICK_1_USB_ID, new Xbox());

		//Driver Controls
		Button shiftDrivetrain = m_driver.getLeftBumper();
		shiftDrivetrain.whenPressed(new DriveSpeedShift(Drive.DriveSpeedShiftState.HI));
        shiftDrivetrain.whenReleased(new DriveSpeedShift(Drive.DriveSpeedShiftState.LO));

        Button intakeLoadDriver = m_driver.getButtonX();
        intakeLoadDriver.whenPressed(new IntakeSetSpeed(Intake.INTAKE_LOAD_SPEED));
        intakeLoadDriver.whenReleased(new IntakeSetSpeed(Intake.INTAKE_HOLD_SPEED));
            
        
		//Operator Controls
		//Intake
	    Button intakeLoad = m_operator.getRightBumper();
        intakeLoad.whenPressed(new IntakeSetSpeed(Intake.INTAKE_LOAD_SPEED));
        intakeLoad.whenReleased(new IntakeSetSpeed(Intake.INTAKE_HOLD_SPEED));
        
        Button intakeLoadSlow = m_operator.getRightTrigger();
        intakeLoadSlow.whenPressed(new IntakeSetSpeed(Intake.INTAKE_LOAD_SLOW_SPEED));
        intakeLoadSlow.whenReleased(new IntakeSetSpeed(Intake.INTAKE_HOLD_SPEED));
		
        Button intakeEject = m_operator.getLeftBumper();
        intakeEject.whenPressed(new IntakeSetSpeed(Intake.INTAKE_EJECT_SPEED));
        intakeEject.whenReleased(new IntakeSetSpeed(0.0));
		
        Button intakeEjectSlow = m_operator.getLeftTrigger();
        intakeEjectSlow.whenPressed(new IntakeSetSpeed(Intake.INTAKE_EJECT_SLOW_SPEED));
        intakeEjectSlow.whenReleased(new IntakeSetSpeed(0.0));
        
        Button ballArmsIn = m_operator.getButtonA();
		ballArmsIn.whenPressed(new IntakeBallArms(BallArmState.IN));

        Button ballArmsOut = m_operator.getButtonY();
        ballArmsOut.whenPressed(new IntakeBallArms(BallArmState.OUT));
        
        Button hatchArmsIn = m_operator.getButtonX();
		hatchArmsIn.whenPressed(new IntakeHatchArms(HatchArmState.IN));

        Button hatchArmsOut = m_operator.getButtonB();
		hatchArmsOut.whenPressed(new IntakeHatchArms(HatchArmState.OUT));
        
        //Elevator
 		Button elevatorShiftHi = m_operator.getDPadUp();
        elevatorShiftHi.whenPressed(new ElevatorSpeedShift(Elevator.ElevatorSpeedShiftState.HI));

        Button elevatorShiftLo = m_operator.getDPadDown();
        elevatorShiftLo.whenPressed(new ElevatorSpeedShift(ElevatorSpeedShiftState.LO));

        Button elevatorJoystickManualMode = m_operator.getDPadLeft();
        elevatorJoystickManualMode.whenPressed(new ElevatorSetMode(ElevatorControlMode.JOYSTICK_MANUAL));

        Button elevatorPidMode = m_operator.getShareButton();								
        elevatorPidMode.whenPressed(new ElevatorSetMode(ElevatorControlMode.JOYSTICK_PID)); 

        Button elevatorReset = m_operator.getOptionsButton();
        elevatorReset.whenPressed(new ElevatorSetZero(Elevator.ZERO_POSITION_INCHES)); 
                				
		// Button intakeCube = m_operator.getButtonA();
		// intakeCube.whenPressed(new IntakeCubeAndLift());

        // Button elevatorMaxPosition = m_operator.getButtonY();
        // elevatorMaxPosition.whenPressed(new ElevatorSetPositionMP(Elevator.MAX_POSITION_INCHES));

        // Button elevatorScalePosition = m_operator.getButtonB();
        // elevatorScalePosition.whenPressed(new ElevatorSetPositionMP(Elevator.SCALE_LOW_POSITION_INCHES));

        // Button elevatorSwitchPosition = m_operator.getButtonX();
        // elevatorSwitchPosition.whenPressed(new ElevatorSetPositionMP(Elevator.SWITCH_POSITION_INCHES));
        
        //Smartdashboard
        Button turnCompressorOff = new InternalButton();
        turnCompressorOff.whenPressed(new TurnCompressorOff());
        SmartDashboard.putData("Compressor Off", turnCompressorOff);

        Button turnCompressorOn = new InternalButton();
        turnCompressorOn.whenPressed(new TurnCompressorOff());
        SmartDashboard.putData("Compressor On", turnCompressorOn);
      
        Button cameraTrack = new InternalButton();
        cameraTrack.whenPressed(new DrivePathCameraTrack(75));
        SmartDashboard.putData("Camera Track", cameraTrack);

        Button cameraTrackStop = new InternalButton();
        cameraTrackStop.whenPressed(new DrivePathCameraTrackStop());
        SmartDashboard.putData("Camera Track Stop", cameraTrackStop);

}

public GameController getDriverController() {
	return m_driver;
}

public GameController getOperatorController() {
	return m_operator;
	}	
}
