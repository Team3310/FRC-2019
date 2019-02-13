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
import frc.team3310.robot.commands.BackLegShift;
import frc.team3310.robot.commands.DriveForwardClimb;
import frc.team3310.robot.commands.DrivePathCameraTrack;
import frc.team3310.robot.commands.DrivePathCameraTrackStop;
import frc.team3310.robot.commands.ElevatorClimbShift;
import frc.team3310.robot.commands.ElevatorSetSpeed;
import frc.team3310.robot.commands.FrontLegShift;
import frc.team3310.robot.commands.TurnCompressorOff;
import frc.team3310.robot.commands.setElevatorMM;
import frc.team3310.robot.controller.GameController;
import frc.team3310.robot.controller.Playstation;
import frc.team3310.robot.subsystems.Elevator.BackLegShiftState;
import frc.team3310.robot.subsystems.Elevator.ElevatorClimbShiftState;
import frc.team3310.robot.subsystems.Elevator.FrontLegShiftState;

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
    m_driver = new GameController(RobotMap.DRIVER_JOYSTICK_1_USB_ID, new Playstation());
    m_operator = new GameController(RobotMap.OPERATOR_JOYSTICK_1_USB_ID, new Playstation());

    // Driver Controls
    // Button ejectHatch = m_driver.getButtonA();
    // ejectHatch.whenPressed(new IntakeHatchArms(HatchArmState.IN));
    // ejectHatch.whenPressed(new IntakeBallArms(BallArmState.OUT));
    // ejectHatch.whenReleased(new IntakeBallArms(BallArmState.IN));

    Button setElevatorMM = m_driver.getButtonY();
    setElevatorMM.whenPressed(new setElevatorMM());

    // Operator Controls
    // Elevator
    // Button intakeBallAndLift = m_operator.getButtonX();
    // intakeBallAndLift.whenPressed(new IntakeBallAndLift());

    // Button elevatorLowHatchPosition = m_operator.getButtonA();
    // elevatorLowHatchPosition.whenPressed(new
    // ElevatorSetPositionMP(Elevator.HATCH_LEVEL_1));
    // elevatorLowHatchPosition.whenPressed(new SetLevel1());

    // Button elevatorMidHatchPosition = m_operator.getButtonB();
    // elevatorMidHatchPosition.whenPressed(new
    // ElevatorSetPositionMP(Elevator.HATCH_LEVEL_2));
    // elevatorMidHatchPosition.whenPressed(new SetLevel2());

    // Button elevatorMaxHatchPosition = m_operator.getButtonY();
    // elevatorMaxHatchPosition.whenPressed(new
    // ElevatorSetPositionMP(Elevator.HATCH_LEVEL_3));
    // elevatorMaxHatchPosition.whenPressed(new SetLevel3());

    // Button elevatorOffsetPosition = m_operator.getButtonPad();
    // elevatorOffsetPosition.whenPressed(new ElevatorSetOffsetMP());

    // Intake
    // Button IntakeHatch = m_operator.getRightBumper();
    // IntakeHatch.whenPressed(new IntakeHatchArms(HatchArmState.OUT));
    // IntakeHatch.whenPressed(new IntakeBallArms(BallArmState.IN));
    // IntakeHatch.whenPressed(new
    // ElevatorSetPositionMP(Elevator.GRAB_HATCH_STATION));
    // IntakeHatch.whenReleased(new IntakeHatchArms(HatchArmState.IN));

    // Button IntakeBallManual = m_operator.getRightTrigger();
    // IntakeBallManual.whenPressed(new IntakeHatchArms(HatchArmState.IN));
    // IntakeBallManual.whenPressed(new IntakeBallArms(BallArmState.OUT));
    // IntakeBallManual.whenPressed(new IntakeSetSpeed(Intake.INTAKE_LOAD_SPEED));
    // IntakeBallManual.whenReleased(new IntakeBallArms(BallArmState.IN));
    // IntakeBallManual.whenReleased(new IntakeSetSpeed(Intake.INTAKE_HOLD_SPEED));

    // Button ejectBall = m_operator.getLeftBumper();
    // ejectBall.whenPressed(new IntakeHatchArms(HatchArmState.OUT));
    // ejectBall.whenPressed(new IntakeSetSpeed(Intake.INTAKE_EJECT_SPEED));
    // ejectBall.whenReleased(new IntakeHatchArms(HatchArmState.IN));
    // ejectBall.whenReleased(new IntakeSetSpeed(0.0));

    // Button ejectBallSlow = m_operator.getLeftTrigger();
    // ejectBallSlow.whenPressed(new IntakeHatchArms(HatchArmState.OUT));
    // ejectBallSlow.whenPressed(new
    // IntakeSetSpeed(Intake.INTAKE_EJECT_SLOW_SPEED));
    // ejectBallSlow.whenReleased(new IntakeHatchArms(HatchArmState.IN));
    // ejectBallSlow.whenReleased(new IntakeSetSpeed(0.0));

    Button driveFowardClimb = m_driver.getButtonA();
    driveFowardClimb.whenPressed(new DriveForwardClimb(.5));
    driveFowardClimb.whenReleased(new DriveForwardClimb(0));

    Button shiftGGG = m_driver.getButtonB();
    shiftGGG.whenPressed(new ElevatorClimbShift(ElevatorClimbShiftState.IN));
    shiftGGG.whenReleased(new ElevatorClimbShift(ElevatorClimbShiftState.OUT));

    Button shiftFrontLeg = m_operator.getButtonA();
    shiftFrontLeg.whenPressed(new FrontLegShift(FrontLegShiftState.OUT));
    shiftFrontLeg.whenReleased(new FrontLegShift(FrontLegShiftState.IN));

    Button shiftBackLeg = m_operator.getButtonB();
    shiftBackLeg.whenPressed(new BackLegShift(BackLegShiftState.OUT));
    shiftBackLeg.whenReleased(new BackLegShift(BackLegShiftState.IN));

    Button lowerGGG = m_operator.getLeftBumper();
    lowerGGG.whenPressed(new ElevatorSetSpeed(.5));
    lowerGGG.whenReleased(new ElevatorSetSpeed(0));

    Button raiseGGG = m_operator.getLeftTrigger();
    raiseGGG.whenPressed(new ElevatorSetSpeed(-.5));
    raiseGGG.whenReleased(new ElevatorSetSpeed(0));

    // Smartdashboard
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
