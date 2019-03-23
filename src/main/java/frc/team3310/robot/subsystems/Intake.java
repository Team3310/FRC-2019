package frc.team3310.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3310.robot.Constants;
import frc.team3310.robot.Robot;
import frc.team3310.robot.RobotMap;
import frc.team3310.utility.lib.drivers.TalonSRXChecker;
import frc.team3310.utility.lib.drivers.TalonSRXFactory;

public class Intake extends Subsystem {
	private static Intake instance;

	public static final double INTAKE_LOAD_SPEED = -.99;
	public static final double INTAKE_LOAD_SLOW_SPEED = -0.4;
	public static final double INTAKE_EJECT_SPEED = 0.8;
	public static final double INTAKE_EJECT_FAST_SPEED = 1.0;
	public static final double INTAKE_EJECT_SLOW_SPEED = 0.3;
	public static final double INTAKE_HOLD_SPEED = -0.15;

	private TalonSRX leftArm;
	private TalonSRX rightArm;

	public static enum BallArmState {
		IN, OUT
	};

	public static enum HatchArmState {
		IN, OUT
	};

	// Sensors
	private DigitalInput frontIRIntakeRightSensor;
	private DigitalInput frontIRIntakeLeftSensor;

	private Solenoid ballArms;
	private Solenoid hatchArms;

	private Intake() {
		try {
			leftArm = TalonSRXFactory.createDefaultTalon(RobotMap.INTAKE_LEFT_CAN_ID);
			leftArm.setNeutralMode(NeutralMode.Brake);
			leftArm.setInverted(true);

			rightArm = TalonSRXFactory.createDefaultTalon(RobotMap.INTAKE_RIGHT_CAN_ID);
			rightArm.setNeutralMode(NeutralMode.Brake);
			rightArm.setInverted(true);

			frontIRIntakeRightSensor = new DigitalInput(RobotMap.INTAKE_FRONT_RIGHT_IR_SENSOR_DIO_ID);
			frontIRIntakeLeftSensor = new DigitalInput(RobotMap.INTAKE_FRONT_LEFT_IR_SENSOR_DIO_ID);

			ballArms = new Solenoid(RobotMap.INTAKE_BALL_ARM_PCM_ID);
			hatchArms = new Solenoid(RobotMap.INTAKE_HATCH_ARM_PCM_ID);

		} catch (Exception e) {
			System.err.println("An error occurred in the Intake constructor");
		}
	}

	public void setBallArmState(BallArmState state) {
		System.out.println("Ball arm state = " + state);
		if (state == BallArmState.IN) {
			ballArms.set(false);
			return;
		}

		Robot.drive.updateLimelight();
		if (isOkToLaunch()) {
			ballArms.set(true);
		} else {
			System.out.println("To far to score");
		}
	}

	private boolean isOkToLaunch() {
		return true; 
		// return !(Robot.elevator.getElevatorInchesOffGround() > Constants.HATCH_LEVEL_2 && Robot.drive.lastValidLimeArea < 9);
	}

	public void setHatchArmState(HatchArmState state) {
		System.out.println("Hatch arm state = " + state);
		if (state == HatchArmState.IN) {
			hatchArms.set(false);
			return;
		}
		if (isOkToLaunch()) {
			hatchArms.set(true);
		}
	}

	@Override

	public void initDefaultCommand() {
	}

	public void setSpeed(double speed) {
		leftArm.set(ControlMode.PercentOutput, speed);
		rightArm.set(ControlMode.PercentOutput, -speed);
	}

	public void setSpeedAsymmetric(double speedLeft, double speedRight) {
		leftArm.set(ControlMode.PercentOutput, speedLeft);
		rightArm.set(ControlMode.PercentOutput, -speedRight);
	}

	public static Intake getInstance() {
		if (instance == null) {
			instance = new Intake();
		}
		return instance;
	}

	public boolean getFrontRightIRIntakeSensor() {
		return frontIRIntakeRightSensor.get();
	}

	public boolean getFrontLeftIRIntakeSensor() {
		return frontIRIntakeLeftSensor.get();
	}

	public synchronized boolean hasBall() {
		return getFrontLeftIRIntakeSensor() && getFrontRightIRIntakeSensor();
	}

	public void updateStatus(Robot.OperationMode operationMode) {
		if (operationMode == Robot.OperationMode.TEST) {
			try {
				SmartDashboard.putNumber("Left Intake Amps", leftArm.getOutputCurrent());
				SmartDashboard.putNumber("Right Intake Amps", rightArm.getOutputCurrent());
				SmartDashboard.putBoolean("Intake Front Right IR Sensor", getFrontRightIRIntakeSensor());
				SmartDashboard.putBoolean("Intake Front Left IR Sensor", getFrontLeftIRIntakeSensor());

			} catch (Exception e) {
			}
		} else if (operationMode == Robot.OperationMode.COMPETITION) {
			SmartDashboard.putBoolean("Intake Front Right IR Sensor", getFrontRightIRIntakeSensor());
			SmartDashboard.putBoolean("Intake Front Left IR Sensor", getFrontLeftIRIntakeSensor());

		}
	}

	public boolean checkSystem() {

		return TalonSRXChecker.CheckTalons(this,

				new ArrayList<TalonSRXChecker.TalonSRXConfig>() {

					{

						add(new TalonSRXChecker.TalonSRXConfig("intake right master", rightArm));

						add(new TalonSRXChecker.TalonSRXConfig("intake left master", leftArm));

					}

				}, new TalonSRXChecker.CheckerConfig() {

					{

						mCurrentFloor = 2;

						mCurrentEpsilon = 2.0;

						mRPMSupplier = null;

					}

				});

	}
}
