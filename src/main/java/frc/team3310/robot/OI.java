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
import frc.team3310.robot.commands.EjectBallFast;
import frc.team3310.robot.commands.EjectBallSlow;
import frc.team3310.robot.commands.EjectBallStop;
import frc.team3310.robot.commands.EjectHatch;
import frc.team3310.robot.commands.ElevatorSetMode;
import frc.team3310.robot.commands.ElevatorSetPositionMM;
import frc.team3310.robot.commands.IntakeBallAndLift;
import frc.team3310.robot.commands.IntakeBallArms;
import frc.team3310.robot.commands.IntakeBallHold;
import frc.team3310.robot.commands.IntakeBallManual;
import frc.team3310.robot.commands.IntakeHatch;
import frc.team3310.robot.commands.IntakeHatchArms;
import frc.team3310.robot.commands.ResetElevatorEncoder;
import frc.team3310.robot.commands.SetRobotClimbBack;
import frc.team3310.robot.commands.SetRobotClimbFront;
import frc.team3310.robot.commands.SetRobotClimbMode;
import frc.team3310.robot.commands.SetRobotScoreMode;
import frc.team3310.robot.commands.TurnCompressorOff;
import frc.team3310.robot.controller.GameController;
import frc.team3310.robot.controller.Xbox;
import frc.team3310.robot.subsystems.Elevator;
import frc.team3310.robot.subsystems.Elevator.ElevatorControlMode;
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
    ejectHatch.whenPressed(new EjectHatch());
    ejectHatch.whenReleased(new IntakeBallArms(BallArmState.IN));

    Button driveFowardClimb = m_driver.getButtonB();
    driveFowardClimb.whenPressed(new DriveForwardClimb(.5));
    driveFowardClimb.whenReleased(new DriveForwardClimb(0));

    Button climbFront = m_driver.getRightTrigger();
    climbFront.whenPressed(new SetRobotClimbFront());

    Button climbBack = m_driver.getLeftTrigger();
    climbBack.whenPressed(new SetRobotClimbBack());

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
    IntakeHatch.whenPressed(new IntakeHatch());
    IntakeHatch.whenReleased(new IntakeHatchArms(HatchArmState.IN));

    Button IntakeBallManual = m_operator.getRightTrigger();
    IntakeBallManual.whenPressed(new IntakeBallManual());
    IntakeBallManual.whenReleased(new IntakeBallHold());

    Button ejectBall = m_operator.getLeftBumper();
    ejectBall.whenPressed(new EjectBallFast());
    ejectBall.whenReleased(new EjectBallStop());

    Button ejectBallSlow = m_operator.getLeftTrigger();
    ejectBallSlow.whenPressed(new EjectBallSlow());
    ejectBallSlow.whenReleased(new EjectBallStop());

    Button setManualMode = m_operator.getOptionsButton();
    setManualMode.whenPressed(new ElevatorSetMode(ElevatorControlMode.JOYSTICK_MANUAL));

    Button IntakeHatchManual = m_operator.getDPadLeft();
    IntakeHatchManual.whenPressed(new IntakeHatchArms(HatchArmState.OUT));
    IntakeHatchManual.whenReleased(new IntakeHatchArms(HatchArmState.IN));

    Button balleLevel1 = m_operator.getDPadDown();
    balleLevel1.whenPressed(new ElevatorSetPositionMM(Elevator.BALL_LEVEL_1));

    Button ballLevel2 = m_operator.getDPadRight();
    ballLevel2.whenPressed(new ElevatorSetPositionMM(Elevator.BALL_LEVEL_2));

    Button ballLevel3 = m_operator.getDPadUp();
    ballLevel3.whenPressed(new ElevatorSetPositionMM(Elevator.BALL_LEVEL_3));

    // Button setClimbUp = m_operator.getDPadUp();
    // setClimbUp.whenPressed(new ElevatorSetSpeed(-.7));
    // setClimbUp.whenReleased(new ElevatorSetSpeed(0));

    // Button setClimbDown = m_operator.getDPadDown();
    // setClimbDown.whenPressed(new ElevatorSetSpeed(.7));
    // setClimbDown.whenReleased(new ElevatorSetSpeed(0));

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
