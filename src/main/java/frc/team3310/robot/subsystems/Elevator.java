package frc.team3310.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3310.robot.Constants;
import frc.team3310.robot.Robot;
import frc.team3310.robot.RobotMap;
import frc.team3310.robot.loops.Loop;
import frc.team3310.utility.MPTalonPIDController;
import frc.team3310.utility.PIDParams;
import frc.team3310.utility.lib.drivers.TalonSRXEncoder;
import frc.team3310.utility.lib.drivers.TalonSRXFactory;

public class Elevator extends Subsystem implements Loop {
	private static Elevator instance;

	public static enum ElevatorControlMode {
		MOTION_PROFILE, JOYSTICK_PID, JOYSTICK_MANUAL, MANUAL
	};

	public static enum ElevatorSpeedShiftState {
		HI, LO
	};

	public static enum FrontLegShiftState {
		IN, OUT
	}

	public static enum BackLegShiftState {
		IN, OUT
	}

	public static enum ElevatorClimbShiftState {
		IN, OUT
	}

	// One revolution of the 30T Drive 1.880" PD pulley = Pi * PD inches = 36/24
	// revs due to pulleys * 34/24 revs due to gears * 36/12 revs due mag encoder
	// gear on ball shifter * 4096 ticks
	public static final double ENCODER_TICKS_TO_INCHES = (36.0 / 12.0) * (36.0 / 24.0) * (34.0 / 24.0) * 4096.0
			/ (1.88 * Math.PI);

	// Defined speeds
	public static final double CLIMB_SPEED = -1.0;
	public static final double TEST_SPEED_UP = 0.3;
	public static final double TEST_SPEED_DOWN = -0.3;
	public static final double AUTO_ZERO_SPEED = -0.3;
	public static final double JOYSTICK_INCHES_PER_MS_HI = 0.75;
	public static final double JOYSTICK_INCHES_PER_MS_LO = JOYSTICK_INCHES_PER_MS_HI / 3.68 * 0.8;

	// Defined positions
	public static final double ZERO_POSITION_AUTON_FORWARD_INCHES = 8.0;
	public static final double ZERO_POSITION_INCHES = -0.25;
	public static final double NEAR_ZERO_POSITION_INCHES = 0.0;
	public static final double MIN_POSITION_INCHES = 0.0;
	public static final double MAX_POSITION_INCHES = 83.4;
	public static final double AFTER_INTAKE_POSITION_INCHES = 4.0;
	public static final double GRAB_HATCH_STATION = 13;
	public static final double HATCH_LEVEL_1 = 13;
	public static final double HATCH_LEVEL_2 = 44;
	public static final double HATCH_LEVEL_3 = 72;
	public static double ROCKET_LEVEL_1;
	public static double ROCKET_LEVEL_2;
	public static double ROCKET_LEVEL_3;
	public static double BALL_LEVEL;
	public static boolean ballScoring;
	public static boolean hatchScoring;
	public boolean Hatch_Level_1;
	public boolean Hatch_Level_2;
	public boolean Hatch_Level_3;

	// Motion profile max velocities and accel times
	public static final double MP_MAX_VELOCITY_INCHES_PER_SEC = 60;
	public static final double MP_T1 = 400; // Fast = 300
	public static final double MP_T2 = 150; // Fast = 150

	// Motor controllers
	private ArrayList<TalonSRXEncoder> motorControllers = new ArrayList<TalonSRXEncoder>();

	private TalonSRXEncoder motor1;
	private TalonSRX motor2;
	private TalonSRX motor3;

	// PID controller and params
	private MPTalonPIDController mpController;

	public static int PID_SLOT = 0;
	public static int MP_SLOT = 1;

	private PIDParams mpPIDParams = new PIDParams(0.2, 0.0, 0.0, 0.0, 0.005, 0.0);
	private PIDParams pidPIDParamsHiGear = new PIDParams(0.075, 0.0, 0.0, 0.0, 0.0, 0.0);
	private PIDParams pidPIDParamsLoGear = new PIDParams(0.45, 0.0, 0.0, 0.0, 0.0, 0.0);
	public static final double KF_UP = 0.005;
	public static final double KF_DOWN = 0.0;
	public static final double PID_ERROR_INCHES = 1.0;
	private long periodMs = (long) (Constants.kLooperDt * 1000.0);

	// Pneumatics
	// private Solenoid speedShift;
	private Solenoid elevatorShift = new Solenoid(RobotMap.ELEVATOR_CLIMB_SHIFT_PCM_ID);
	private Solenoid frontLegShift = new Solenoid(RobotMap.FRONT_LEG_SHIFT_PCM_ID);
	private Solenoid backLegShift = new Solenoid(RobotMap.BACK_LEG_SHIFT_PCM_ID);

	private ElevatorSpeedShiftState shiftState = ElevatorSpeedShiftState.LO; // Default position
	private FrontLegShiftState frontShiftState = FrontLegShiftState.IN;
	private BackLegShiftState backShiftState = BackLegShiftState.IN;
	private ElevatorClimbShiftState elevatorClimbShiftState = ElevatorClimbShiftState.OUT;

	// Misc
	public static final double AUTO_ZERO_MOTOR_CURRENT = 4.0;
	private boolean isFinished;
	private ElevatorControlMode elevatorControlMode = ElevatorControlMode.JOYSTICK_MANUAL;
	private double targetPositionInchesPID = 0;
	private boolean firstMpPoint;
	private double joystickInchesPerMs = JOYSTICK_INCHES_PER_MS_LO;
	public boolean toLow;

	private Elevator() {
		try {
			motor1 = TalonSRXFactory.createTalonEncoder(RobotMap.ELEVATOR_MOTOR_1_CAN_ID, ENCODER_TICKS_TO_INCHES,
					false, FeedbackDevice.QuadEncoder);
			motor2 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.ELEVATOR_MOTOR_2_CAN_ID,
					RobotMap.ELEVATOR_MOTOR_1_CAN_ID);
			motor3 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.ELEVATOR_MOTOR_3_CAN_ID,
					RobotMap.ELEVATOR_MOTOR_1_CAN_ID);

			motor1.setInverted(true);
			motor2.setInverted(true);
			motor3.setInverted(true);

			// if (motor1.isSensorPresent(CANTalon.FeedbackDevice.QuadEncoder) !=
			// CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
			// Driver.reportError("Could not detect elevator motor 1 encoder encoder!",
			// false);
			// }

			motorControllers.add(motor1);

			// speedShift = new Solenoid(RobotMap.ELEVATOR_SPEEDSHIFT_PCM_ID);
		} catch (Exception e) {
			System.err.println("An error occurred in the DriveTrain constructor");
		}
	}

	@Override
	public void initDefaultCommand() {
	}

	public void resetZeroPosition(double position) {
		mpController.resetZeroPosition(position);
	}

	private synchronized void setElevatorControlMode(ElevatorControlMode controlMode) {
		this.elevatorControlMode = controlMode;
	}

	private synchronized ElevatorControlMode getElevatorControlMode() {
		return this.elevatorControlMode;
	}

	public void setSpeed(double speed) {
		motor1.set(ControlMode.PercentOutput, speed);
		setElevatorControlMode(ElevatorControlMode.MANUAL);
	}

	public void setSpeedJoystick(double speed) {
		motor1.set(ControlMode.PercentOutput, speed);
		setElevatorControlMode(ElevatorControlMode.JOYSTICK_MANUAL);
	}

	public void setPositionPID(double targetPositionInches) {
		mpController.setPIDSlot(PID_SLOT);
		updatePositionPID(targetPositionInches);
		setElevatorControlMode(ElevatorControlMode.JOYSTICK_PID);
		setFinished(false);
	}

	public void updatePositionPID(double targetPositionInches) {
		targetPositionInchesPID = limitPosition(targetPositionInches);
		double startPositionInches = motor1.getPositionWorld();
		mpController.setTarget(targetPositionInchesPID,
				targetPositionInchesPID > startPositionInches ? KF_UP : KF_DOWN);
	}

	public void setPositionMP(double targetPositionInches) {
		double startPositionInches = motor1.getPositionWorld();
		mpController.setMPTarget(startPositionInches, limitPosition(targetPositionInches),
				MP_MAX_VELOCITY_INCHES_PER_SEC, MP_T1, MP_T2);
		setFinished(false);
		firstMpPoint = true;
		setElevatorControlMode(ElevatorControlMode.MOTION_PROFILE);
	}

	private double limitPosition(double targetPosition) {
		if (targetPosition < MIN_POSITION_INCHES) {
			return MIN_POSITION_INCHES;
		} else if (targetPosition > MAX_POSITION_INCHES) {
			return MAX_POSITION_INCHES;
		}

		return targetPosition;
	}

	@Override
	public void onStart(double timestamp) {
		mpController = new MPTalonPIDController(periodMs, motorControllers);
		mpController.setPID(mpPIDParams, MP_SLOT);
		mpController.setPID(pidPIDParamsHiGear, PID_SLOT);
		mpController.setPIDSlot(PID_SLOT);
		mpController.setPID(pidPIDParamsHiGear, PID_SLOT);
		mpController.setPIDSlot(PID_SLOT);
	}

	@Override
	public void onStop(double timestamp) {
		// TODO Auto-generated method stub

	}

	@Override
	public void onLoop(double timestamp) {
		synchronized (Elevator.this) {
			switch (getElevatorControlMode()) {
			case JOYSTICK_PID:
				controlPidWithJoystick();
				// checkInchesIntake();
				break;
			case JOYSTICK_MANUAL:
				controlManualWithJoystick();
				// checkInchesIntake();
				break;
			case MOTION_PROFILE:
				setScoringLocation();
				checkScoringPosition();
				if (!isFinished()) {
					if (firstMpPoint) {
						mpController.setPIDSlot(MP_SLOT);
						firstMpPoint = false;
					}
					setFinished(mpController.controlLoopUpdate());
					if (isFinished()) {
						mpController.setPIDSlot(PID_SLOT);
					}
				}
				break;
			default:
				break;
			}
		}
	}

	private void controlPidWithJoystick() {
		double joystickPosition = -Robot.oi.getOperatorController().getLeftYAxis();
		double deltaPosition = joystickPosition * joystickInchesPerMs;
		targetPositionInchesPID = targetPositionInchesPID + deltaPosition;
		updatePositionPID(targetPositionInchesPID);
	}

	private void controlManualWithJoystick() {
		double joyStickSpeed = -Robot.oi.getOperatorController().getLeftYAxis();
		setSpeedJoystick(joyStickSpeed);
	}

	private void checkScoringPosition() {
		if (Hatch_Level_1 == true && Hatch_Level_2 == false && Hatch_Level_3 == false) {
			BALL_LEVEL = 22;
		} else if (Hatch_Level_1 == false && Hatch_Level_2 == true && Hatch_Level_3 == false) {
			BALL_LEVEL = 50;
		}

		else if (Hatch_Level_1 == false && Hatch_Level_2 == false && Hatch_Level_3 == true) {
			BALL_LEVEL = 80;
		}
	}

	private void setScoringLocation() {
		if (hatchScoring == true && ballScoring == false) {
			ROCKET_LEVEL_1 = 13;
			ROCKET_LEVEL_2 = 44;
			ROCKET_LEVEL_3 = 75;
		} else if (hatchScoring == false && ballScoring == true) {
			ROCKET_LEVEL_1 = 21.5;
			ROCKET_LEVEL_2 = 50;
			ROCKET_LEVEL_3 = 80;
		}
	}

	// private void checkInchesIntake() {
	// if (targetPositionInchesPID < 5) {
	// toLow = true;
	// } else {
	// toLow = false;
	// }
	// }

	// public void setShiftState(ElevatorSpeedShiftState state) {
	// shiftState = state;
	// if (state == ElevatorSpeedShiftState.HI) {
	// joystickInchesPerMs = JOYSTICK_INCHES_PER_MS_HI;
	// speedShift.set(true);
	// mpController.setPID(pidPIDParamsHiGear, PID_SLOT);
	// } else if (state == ElevatorSpeedShiftState.LO) {
	// joystickInchesPerMs = JOYSTICK_INCHES_PER_MS_LO;
	// speedShift.set(false);
	// mpController.setPID(pidPIDParamsLoGear, PID_SLOT);
	// }
	// }

	public void setFrontLegState(FrontLegShiftState state) {
		frontShiftState = state;
		if (state == FrontLegShiftState.IN) {
			frontLegShift.set(false);
			System.out.println("FL IN");
		} else if (state == FrontLegShiftState.OUT) {
			frontLegShift.set(true);
			System.out.println("FL OUT");
		}
	}

	public void setBackLegState(BackLegShiftState state) {
		backShiftState = state;
		if (state == BackLegShiftState.IN) {
			backLegShift.set(false);
			System.out.println("BL IN");
		} else if (state == BackLegShiftState.OUT) {
			backLegShift.set(true);
			System.out.println("BL OUT");
		}
	}

	public void setElevatorClimbState(ElevatorClimbShiftState state) {
		elevatorClimbShiftState = state;
		if (state == ElevatorClimbShiftState.IN) {
			elevatorShift.set(false);
			System.out.println("GG IN");
		} else if (state == ElevatorClimbShiftState.OUT) {
			elevatorShift.set(true);
			System.out.println("GG OUT");
		}
	}

	public ElevatorSpeedShiftState getShiftState() {
		return shiftState;
	}

	public FrontLegShiftState getFrontShiftState() {
		return frontShiftState;
	}

	public BackLegShiftState getBackShiftState() {
		return backShiftState;
	}

	public ElevatorClimbShiftState getClimbShiftState() {
		return elevatorClimbShiftState;
	}

	public double getPositionInches() {
		return motor1.getPositionWorld();
	}

	// public double getAverageMotorCurrent() {
	// return (Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_1_CAN_ID) +
	// Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_2_CAN_ID) +
	// Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_3_CAN_ID)) / 3;
	// }

	public double getAverageMotorCurrent() {
		return (motor1.getOutputCurrent() + motor2.getOutputCurrent() + motor3.getOutputCurrent()) / 3;
	}

	public void getElevatorLevel() {

	}

	public synchronized boolean isFinished() {
		return isFinished;
	}

	public synchronized void setFinished(boolean isFinished) {
		this.isFinished = isFinished;
	}

	public double getPeriodMs() {
		return periodMs;
	}

	public void updateStatus(Robot.OperationMode operationMode) {
		if (operationMode == Robot.OperationMode.TEST) {
			try {
				SmartDashboard.putNumber("Elevator Position Inches", motor1.getPositionWorld());
				SmartDashboard.putNumber("Elevator Motor 1 Amps", motor1.getOutputCurrent());
				SmartDashboard.putNumber("Elevator Motor 2 Amps", motor2.getOutputCurrent());
				SmartDashboard.putNumber("Elevator Motor 3 Amps", motor3.getOutputCurrent());
				SmartDashboard.putNumber("Elevator Average Amps", getAverageMotorCurrent());
				// SmartDashboard.putNumber("Elevator Motor 1 Amps PDP",
				// Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_1_CAN_ID));
				// SmartDashboard.putNumber("Elevator Motor 2 Amps PDP",
				// Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_2_CAN_ID));
				// SmartDashboard.putNumber("Elevator Motor 3 Amps PDP",
				// Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_3_CAN_ID));
				SmartDashboard.putNumber("Elevator Target PID Position", targetPositionInchesPID);
			} catch (Exception e) {
			}
		}
	}

	public static Elevator getInstance() {
		if (instance == null) {
			instance = new Elevator();
		}
		return instance;
	}
}