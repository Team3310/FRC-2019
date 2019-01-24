package frc.team3310.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3310.robot.Robot;
import frc.team3310.robot.RobotMap;
import frc.team3310.utility.lib.drivers.TalonSRXChecker;
import frc.team3310.utility.lib.drivers.TalonSRXFactory;

public class Intake extends Subsystem {
	private static Intake instance;

	public static final double INTAKE_REAR_EJECT_FAST_SPEED = 1.0;
	public static final double INTAKE_REAR_EJECT_MEDIUM_SPEED = 0.8; //0.7 //0.8
	public static final double INTAKE_LOAD_SPEED = -0.7;
	public static final double INTAKE_LOAD_SLOW_SPEED = -0.4;
	public static final double INTAKE_EJECT_SPEED = 0.8;
	public static final double INTAKE_EJECT_FAST_SPEED = 1.0; //Intake Eject Speed for first cube APR
	public static final double INTAKE_EJECT_SLOW_SPEED = 0.4;
	public static final double INTAKE_ADJUST_SPEED = 0.3;
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
	private DigitalInput frontIRIntakeSensor;
	private DigitalInput frontLeftVEXIntakeSensor;
	private DigitalInput frontRightVEXIntakeSensor;
	private DigitalInput backIntakeSensor;

	private Solenoid ballArms;
	private Solenoid hatchArms;
	
	private Intake() {
		try {
			leftArm = TalonSRXFactory.createDefaultTalon(RobotMap.INTAKE_LEFT_CAN_ID);
			leftArm.setNeutralMode(NeutralMode.Brake);
			leftArm.setInverted(true);
			
			rightArm = TalonSRXFactory.createDefaultTalon(RobotMap.INTAKE_RIGHT_CAN_ID);
			rightArm.setNeutralMode(NeutralMode.Brake);

			frontIRIntakeSensor = new DigitalInput(RobotMap.INTAKE_FRONT_IR_SENSOR_DIO_ID);
			frontLeftVEXIntakeSensor = new DigitalInput(RobotMap.INTAKE_FRONT_LEFT_VEX_SENSOR_DIO_ID);
			frontRightVEXIntakeSensor = new DigitalInput(RobotMap.INTAKE_FRONT_RIGHT_VEX_SENSOR_DIO_ID);
			backIntakeSensor = new DigitalInput(RobotMap.INTAKE_BACK_IR_SENSOR_DIO_ID);		
			
			ballArms = new Solenoid(7);
			hatchArms = new Solenoid(6);

		}
		catch (Exception e) {
			System.err.println("An error occurred in the Intake constructor");
		}
	}

	public void setBallArmState(BallArmState state) {
		System.out.println("Ball arm state = " + state);
		if(state == BallArmState.IN) {
			ballArms.set(false);
		}
		else if(state == BallArmState.OUT) {
			ballArms.set(true);
		}
	}

	public void setHatchArmState(HatchArmState state) {
		System.out.println("Hatch arm state = " + state);
		if(state == HatchArmState.IN) {
			hatchArms.set(false);
		}
		else if(state == HatchArmState.OUT) {
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
		if(instance == null) {
			instance = new Intake();
		}
		return instance;
	}

	public boolean getFrontIRIntakeSensor() {
		return frontIRIntakeSensor.get();
	}
	
	public boolean getFrontLeftVEXIntakeSensor() {
		return !frontLeftVEXIntakeSensor.get();   
	}
	
	public boolean getFrontRightVEXIntakeSensor() {
		return !frontRightVEXIntakeSensor.get();   
	}

	public boolean getBackIntakeSensor() {
		return backIntakeSensor.get();
	}

	
	
	public void updateStatus(Robot.OperationMode operationMode) {
		if (operationMode == Robot.OperationMode.TEST) {
			try {
				SmartDashboard.putBoolean("Intake Front Left VEX Sensor", getFrontLeftVEXIntakeSensor());
				SmartDashboard.putBoolean("Intake Front Right VEX Sensor", getFrontRightVEXIntakeSensor());
				SmartDashboard.putBoolean("Intake Front IR Sensor", getFrontIRIntakeSensor());
				SmartDashboard.putBoolean("Intake Back Sensor", getBackIntakeSensor());
				SmartDashboard.putNumber("Left Intake Amps", leftArm.getOutputCurrent());
				SmartDashboard.putNumber("Right Intake Amps", rightArm.getOutputCurrent());
			}
			catch (Exception e) {
			}
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