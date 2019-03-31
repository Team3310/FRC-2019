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
import frc.team3310.auto.commands.DriveRelativeTurnMP;
import frc.team3310.robot.commands.EjectBallFast;
import frc.team3310.robot.commands.EjectBallSlow;
import frc.team3310.robot.commands.EjectBallStop;
import frc.team3310.robot.commands.EjectHatch;
import frc.team3310.robot.commands.ElevatorAutoZero;
import frc.team3310.robot.commands.ElevatorAutoZeroSensor;
import frc.team3310.robot.commands.ElevatorClimbBoost;
import frc.team3310.robot.commands.ElevatorClimbEndGameExtra;
import frc.team3310.robot.commands.ElevatorClimbEndGameLvl2;
import frc.team3310.robot.commands.ElevatorClimbEndGameLvl3;
import frc.team3310.robot.commands.ElevatorHatchLevel;
import frc.team3310.robot.commands.ElevatorSetMode;
import frc.team3310.robot.commands.ElevatorSetPositionMM;
import frc.team3310.robot.commands.IntakeBallAndLift;
import frc.team3310.robot.commands.IntakeBallArms;
import frc.team3310.robot.commands.IntakeBallHold;
import frc.team3310.robot.commands.IntakeBallManual;
import frc.team3310.robot.commands.IntakeHatch;
import frc.team3310.robot.commands.IntakeHatchArms;
import frc.team3310.robot.commands.OverrideClimb;
import frc.team3310.robot.commands.ResetSensor;
import frc.team3310.robot.commands.SetRobotClimbBack;
import frc.team3310.robot.commands.SetRobotClimbFront;
import frc.team3310.robot.commands.SetRobotClimbMode;
import frc.team3310.robot.commands.SetRobotScoreMode;
import frc.team3310.robot.commands.SwitchCameraPipeline;
import frc.team3310.robot.commands.TurnCompressorOff;
import frc.team3310.robot.commands.TurnCompressorOn;
import frc.team3310.robot.controller.GameController;
import frc.team3310.robot.controller.Xbox;
import frc.team3310.robot.subsystems.Elevator.ElevatorControlMode;
import frc.team3310.robot.subsystems.Intake.BallArmState;
import frc.team3310.robot.subsystems.Intake.HatchArmState;
import frc.team3310.utility.MPSoftwarePIDController.MPSoftwareTurnType;

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
    ejectHatch.whenReleased(new IntakeHatchArms(HatchArmState.IN));

    Button climbLvl3 = m_driver.getButtonY();
    climbLvl3.whenPressed(new ElevatorClimbEndGameLvl3());

    Button climbLvl2 = m_driver.getButtonB();
    climbLvl2.whenPressed(new ElevatorClimbEndGameLvl2());

    Button climbBoost = m_driver.getButtonX();
    climbBoost.whenPressed(new ElevatorClimbBoost());

    Button climbFront = m_driver.getDPadDown();
    climbFront.whenPressed(new SetRobotClimbFront());

    Button climbBack = m_driver.getDPadUp();
    climbBack.whenPressed(new SetRobotClimbBack());

    Button climbState = m_driver.getDPadRight();
    climbState.whenPressed(new SetRobotClimbMode());

    Button scoreState = m_driver.getDPadLeft();
    scoreState.whenPressed(new SetRobotScoreMode());

    Button IntakeHatchManualD = m_driver.getLeftTrigger();
    IntakeHatchManualD.whenReleased(new IntakeHatchArms(HatchArmState.IN));
    IntakeHatchManualD.whenReleased(new ElevatorHatchLevel());

    Button switchCameraPipeline = m_driver.getRightTrigger();
    switchCameraPipeline.whenPressed(new SwitchCameraPipeline(0));
    switchCameraPipeline.whenReleased(new SwitchCameraPipeline(1));

    Button overrideClimb = m_driver.getShareButton();
    overrideClimb.whenPressed(new OverrideClimb());

    Button climbLvl2ToLvl3 = m_driver.getOptionsButton();
    climbLvl2ToLvl3.whenPressed(new ElevatorClimbEndGameExtra());

    // Operator Controls
    // Elevator
    Button intakeBallAndLift = m_operator.getButtonX();
    intakeBallAndLift.whenPressed(new IntakeBallAndLift());

    Button elevatorLowHatchPosition = m_operator.getButtonA();
    elevatorLowHatchPosition.whenPressed(new ElevatorSetPositionMM(Constants.HATCH_LEVEL_1));

    Button elevatorMidHatchPosition = m_operator.getButtonB();
    elevatorMidHatchPosition.whenPressed(new ElevatorSetPositionMM(Constants.HATCH_LEVEL_2));

    Button elevatorMaxHatchPosition = m_operator.getButtonY();
    elevatorMaxHatchPosition.whenPressed(new ElevatorSetPositionMM(Constants.HATCH_LEVEL_3));

    // Intake
    Button IntakeHatch = m_operator.getRightBumper();
    IntakeHatch.whenPressed(new IntakeHatch());

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

    Button IntakeHatchManual = m_operator.getShareButton();
    IntakeHatchManual.whenPressed(new IntakeHatchArms(HatchArmState.OUT));
    IntakeHatchManual.whenReleased(new IntakeHatchArms(HatchArmState.IN));
    IntakeHatchManual.whenReleased(new ElevatorHatchLevel());

    Button balleLevel1 = m_operator.getDPadDown();
    balleLevel1.whenPressed(new ElevatorSetPositionMM(Constants.BALL_LEVEL_1));

    Button ballLevel2 = m_operator.getDPadRight();
    ballLevel2.whenPressed(new ElevatorSetPositionMM(Constants.BALL_LEVEL_2));

    Button ballLevel3 = m_operator.getDPadUp();
    ballLevel3.whenPressed(new ElevatorSetPositionMM(Constants.BALL_LEVEL_3));

    Button ballLevelCargo = m_operator.getDPadLeft();
    ballLevelCargo.whenPressed(new ElevatorSetPositionMM(Constants.BALL_LEVEL_CARGO));

    Button autoZero = m_operator.getLeftJoystickButton();
    autoZero.whenPressed(new ElevatorAutoZero(true));

    Button sensorZero = m_operator.getRightJoystickButton();
    sensorZero.whenPressed(new ElevatorAutoZeroSensor());

    // Smartdashboard
    Button turnCompressorOff = new InternalButton();
    turnCompressorOff.whenPressed(new TurnCompressorOff());
    SmartDashboard.putData("Compressor Off", turnCompressorOff);

    Button turnCompressorOn = new InternalButton();
    turnCompressorOn.whenPressed(new TurnCompressorOn());
    SmartDashboard.putData("Compressor On", turnCompressorOn);

    Button resetSensors = new InternalButton();
    resetSensors.whenPressed(new ResetSensor());
    SmartDashboard.putData("Reset Sensor", resetSensors);

    Button turnRelative90 = new InternalButton();
    turnRelative90.whenPressed(new DriveRelativeTurnMP(-90, 240, MPSoftwareTurnType.TANK));
    SmartDashboard.putData("Turn Relative 90", turnRelative90);

  }

  public GameController getDriverController() {
    return m_driver;
  }

  public GameController getOperatorController() {
    return m_operator;
  }
}
