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
import frc.team3310.robot.commands.ElevatorSetPositionMP;
import frc.team3310.robot.commands.IntakeBallAndLift;
import frc.team3310.robot.commands.IntakeBallArms;
import frc.team3310.robot.commands.IntakeHatchArms;
import frc.team3310.robot.commands.IntakeSetSpeed;
import frc.team3310.robot.commands.TurnCompressorOff;
import frc.team3310.robot.controller.GameController;
import frc.team3310.robot.controller.Xbox;
import frc.team3310.robot.subsystems.Drive.DriveSpeedShiftState;
import frc.team3310.robot.subsystems.Elevator;
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
        Button shiftSpeed = m_driver.getRightBumper();
        shiftSpeed.whenPressed(new DriveSpeedShift(DriveSpeedShiftState.HI));
        shiftSpeed.whenReleased(new DriveSpeedShift(DriveSpeedShiftState.LO));

        Button ejectHatch = m_driver.getButtonA();
        ejectHatch.whenPressed(new IntakeHatchArms(HatchArmState.IN));
        ejectHatch.whenPressed(new IntakeBallArms(BallArmState.OUT));
        ejectHatch.whenReleased(new IntakeBallArms(BallArmState.IN));
              
        //Operator Controls
        //Elevator
        Button intakeBallAndLift = m_operator.getButtonX();
        intakeBallAndLift.whenPressed(new IntakeBallAndLift());

        Button elevatorLowBallPosition = m_operator.getButtonA();
        elevatorLowBallPosition.whenPressed(new ElevatorSetPositionMP(Elevator.ROCKET_LEVEL_1));

        Button elevatorMidBallPosition = m_operator.getButtonB();
        elevatorMidBallPosition.whenPressed(new ElevatorSetPositionMP(Elevator.ROCKET_LEVEL_2));

        Button elevatorMaxBallPosition = m_operator.getButtonY();
        elevatorMaxBallPosition.whenPressed(new ElevatorSetPositionMP(Elevator.ROCKET_LEVEL_3));
 
        //Intake
        Button IntakeHatch = m_operator.getRightBumper();
        IntakeHatch.whenPressed(new IntakeHatchArms(HatchArmState.OUT));
        IntakeHatch.whenPressed(new ElevatorSetPositionMP(Elevator.GRAB_HATCH_STATION));
        IntakeHatch.whenReleased(new IntakeHatchArms(HatchArmState.IN));

        Button IntakeBallManual = m_operator.getRightTrigger();
        IntakeBallManual.whenPressed(new IntakeBallArms(BallArmState.OUT));
        IntakeBallManual.whenPressed(new IntakeSetSpeed(Intake.INTAKE_LOAD_SPEED));
        IntakeBallManual.whenReleased(new IntakeBallArms(BallArmState.IN));
        IntakeBallManual.whenReleased(new IntakeSetSpeed(Intake.INTAKE_HOLD_SPEED));

        Button ejectBall = m_operator.getLeftBumper();
        ejectBall.whenPressed(new IntakeHatchArms(HatchArmState.OUT));
        ejectBall.whenPressed(new IntakeSetSpeed(Intake.INTAKE_EJECT_SPEED));
        ejectBall.whenReleased(new IntakeHatchArms(HatchArmState.IN));
        ejectBall.whenReleased(new IntakeSetSpeed(0.0));

        Button ejectBallSlow = m_operator.getLeftTrigger();
        ejectBallSlow.whenPressed(new IntakeHatchArms(HatchArmState.OUT));
        ejectBallSlow.whenPressed(new IntakeSetSpeed(Intake.INTAKE_EJECT_SLOW_SPEED));
        ejectBallSlow.whenReleased(new IntakeHatchArms(HatchArmState.IN));
        ejectBallSlow.whenReleased(new IntakeSetSpeed(0.0));
                
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
