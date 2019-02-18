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
import frc.team3310.robot.commands.DriveForwardClimb;
import frc.team3310.robot.commands.ElevatorSetMode;
import frc.team3310.robot.commands.ElevatorSetPositionMM;
import frc.team3310.robot.commands.ElevatorSetPositionMP;
import frc.team3310.robot.commands.ElevatorSetSpeed;
import frc.team3310.robot.commands.IntakeBallAndLift;
import frc.team3310.robot.commands.IntakeBallArms;
import frc.team3310.robot.commands.IntakeHatchArms;
import frc.team3310.robot.commands.IntakeSetSpeed;
import frc.team3310.robot.commands.ResetElevatorEncoder;
import frc.team3310.robot.commands.SetRobotClimbMode;
import frc.team3310.robot.commands.SetRobotScoreMode;
import frc.team3310.robot.commands.TurnCompressorOff;
import frc.team3310.robot.controller.GameController;
import frc.team3310.robot.controller.Xbox;
import frc.team3310.robot.subsystems.Elevator;
import frc.team3310.robot.subsystems.Elevator.ElevatorControlMode;
import frc.team3310.robot.subsystems.Intake;
import frc.team3310.robot.subsystems.Intake.BallArmState;
import frc.team3310.robot.subsystems.Intake.HatchArmState;

public class OI {

  private static OI instance;

  private GameController m_driver;
  private GameController m_operator;

  public static OI getInstance() {
    if (instance == null) {
      instance = new OI();
    }
    return instance;
  }

  private OI() {
    // Driver controller
    m_driver = new GameController(RobotMap.DRIVER_JOYSTICK_1_USB_ID, new Xbox());
    m_operator = new GameController(RobotMap.OPERATOR_JOYSTICK_1_USB_ID, new Xbox());

    // Driver Controls
    Button ejectHatch = m_driver.getButtonA();
    ejectHatch.whenPressed(new IntakeHatchArms(HatchArmState.IN));
    ejectHatch.whenPressed(new IntakeBallArms(BallArmState.OUT));
    ejectHatch.whenReleased(new IntakeBallArms(BallArmState.IN));

    Button driveFowardClimb = m_driver.getButtonB();
    driveFowardClimb.whenPressed(new DriveForwardClimb(.5));
    driveFowardClimb.whenReleased(new DriveForwardClimb(0));

    Button climbState = m_driver.getRightBumper();
    climbState.whenPressed(new SetRobotClimbMode());

    Button scoreState = m_driver.getLeftBumper();
    scoreState.whenPressed(new SetRobotScoreMode());

    // Operator Controls
    // Elevator
    Button intakeBallAndLift = m_operator.getButtonX();
    intakeBallAndLift.whenPressed(new IntakeBallAndLift());

    Button elevatorLowHatchPosition = m_operator.getButtonA();
    elevatorLowHatchPosition.whenPressed(new ElevatorSetPositionMM(Elevator.HATCH_LEVEL_1));

    Button elevatorMidHatchPosition = m_operator.getButtonB();
    elevatorMidHatchPosition.whenPressed(new ElevatorSetPositionMM(Elevator.HATCH_LEVEL_2));

    Button elevatorMaxHatchPosition = m_operator.getButtonY();
    elevatorMaxHatchPosition.whenPressed(new ElevatorSetPositionMM(Elevator.HATCH_LEVEL_3));

    // Intake
    Button IntakeHatch = m_operator.getRightBumper();
    IntakeHatch.whenPressed(new IntakeHatchArms(HatchArmState.OUT));
    IntakeHatch.whenPressed(new IntakeBallArms(BallArmState.IN));
    IntakeHatch.whenPressed(new ElevatorSetPositionMP(Elevator.GRAB_HATCH_STATION));
    IntakeHatch.whenReleased(new IntakeHatchArms(HatchArmState.IN));

    Button IntakeBallManual = m_operator.getRightTrigger();
    IntakeBallManual.whenPressed(new IntakeHatchArms(HatchArmState.IN));
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

    Button setManualMode = m_operator.getStartButton();
    setManualMode.whenPressed(new ElevatorSetMode(ElevatorControlMode.JOYSTICK_MANUAL));

    Button IntakeHatchManual = m_operator.getDPadLeft();
    IntakeHatchManual.whenPressed(new IntakeHatchArms(HatchArmState.OUT));
    IntakeHatchManual.whenReleased(new IntakeHatchArms(HatchArmState.IN));

    Button climbUp = m_operator.getDPadUp();
    climbUp.whenPressed(new ElevatorSetSpeed(-.6));
    climbUp.whenReleased(new ElevatorSetSpeed(0));

    Button climbDown = m_operator.getDPadDown();
    climbDown.whenPressed(new ElevatorSetSpeed(.6));
    climbDown.whenReleased(new ElevatorSetSpeed(0));

    // Smartdashboard
    Button turnCompressorOff = new InternalButton();
    turnCompressorOff.whenPressed(new TurnCompressorOff());
    SmartDashboard.putData("Compressor Off", turnCompressorOff);

    Button turnCompressorOn = new InternalButton();
    turnCompressorOn.whenPressed(new TurnCompressorOff());
    SmartDashboard.putData("Compressor On", turnCompressorOn);

    Button resetElevatorEncoders = new InternalButton();
    resetElevatorEncoders.whenPressed(new ResetElevatorEncoder());
    SmartDashboard.putData("Reset Elevator Encoder", resetElevatorEncoders);

    // Button cameraTrack = new InternalButton();
    // cameraTrack.whenPressed(new DrivePathCameraTrack(75));
    // SmartDashboard.putData("Camera Track", cameraTrack);

    // Button cameraTrackStop = new InternalButton();
    // cameraTrackStop.whenPressed(new DrivePathCameraTrackStop());
    // SmartDashboard.putData("Camera Track Stop", cameraTrackStop);

  }

  public GameController getDriverController() {
    return m_driver;
  }

  public GameController getOperatorController() {
    return m_operator;
  }
}
