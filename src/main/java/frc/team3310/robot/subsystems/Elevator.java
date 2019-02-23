package frc.team3310.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3310.robot.Constants;
import frc.team3310.robot.Robot;
import frc.team3310.robot.RobotMap;
import frc.team3310.robot.loops.Loop;
import frc.team3310.utility.Util;
import frc.team3310.utility.lib.drivers.TalonSRXEncoder;
import frc.team3310.utility.lib.drivers.TalonSRXFactory;
import frc.team3310.utility.lib.drivers.TalonSRXUtil;

public class Elevator extends Subsystem implements Loop {
	private static Elevator instance;

	public static enum ElevatorControlMode {
		MOTION_MAGIC, JOYSTICK_POSITION_PID, JOYSTICK_MANUAL, MANUAL
	};

	public static enum FrontLegShiftState {
		LOCKED, ENGAGED
	}

	public static enum BackLegShiftState {
		LOCKED, ENGAGED
	}

	public static enum ElevatorClimbShiftState {
		LOCKED, ENGAGED
	}

	public static final double INCHES_TO_ENCODER_TICKS_ELEVATOR = (50.0 / 50.0) * (34.0 / 34.0) * 4096.0 / (1.2987013 * Math.PI);
	public static final double INCHES_TO_ENCODER_TICKS_GGG = (50.0 / 50.0) * (50.0 / 18.0) * (40.0 / 24.0) * 4096.0 / (1.128 * Math.PI);

	// Defined speeds
	public static final double TEST_SPEED_UP = 0.3;
	public static final double TEST_SPEED_DOWN = -0.2;
	public static final double AUTO_ZERO_SPEED = -0.2;
	public static final double JOYSTICK_TICKS_PER_MS_ELEVATOR = 0.5 * INCHES_TO_ENCODER_TICKS_ELEVATOR;
	public static final double JOYSTICK_TICKS_PER_MS_GGG = JOYSTICK_TICKS_PER_MS_ELEVATOR / INCHES_TO_ENCODER_TICKS_ELEVATOR * INCHES_TO_ENCODER_TICKS_GGG * 0.8;

	// Motor controllers
	private TalonSRXEncoder motor1;
	private TalonSRX motor2;
	private TalonSRX motor3;

	// Pneumatics
	private Solenoid elevatorShift;
	private Solenoid frontLegShift;
	private Solenoid backLegShift;

	//Sensors
	// private DigitalInput maxRevElevatorSensor;
	// private DigitalInput minRevElevatorSensor;

	// Position PID
	private static final int kPositionPIDSlot = 0;

	// Motion Magic
	private static final int kMotionMagicSlot = 1;

	// Shifting Default
	private FrontLegShiftState frontShiftState = FrontLegShiftState.LOCKED; // ENGAGED = Climb Mode
	private BackLegShiftState backShiftState = BackLegShiftState.LOCKED; // ENGAGED = Climb Mode
	private ElevatorClimbShiftState elevatorClimbShiftState = ElevatorClimbShiftState.ENGAGED; // LOCKED = Climb Mode

	// Misc
	public static final double AUTO_ZERO_MOTOR_CURRENT = 1.0;
	private double joystickTicksPerMs = JOYSTICK_TICKS_PER_MS_ELEVATOR;
	private double currentInchesToEncoderTicks = INCHES_TO_ENCODER_TICKS_ELEVATOR;
	private double targetPositionTicks = 0;
	private double joyStickSpeed;
	public boolean elevatorCargoMode = false;

	private ElevatorControlMode elevatorControlMode = ElevatorControlMode.MOTION_MAGIC;

	// Constructor
	private Elevator() {
		try {
			motor1 = TalonSRXFactory.createTalonEncoder(RobotMap.ELEVATOR_MOTOR_1_CAN_ID, currentInchesToEncoderTicks,
					false, FeedbackDevice.QuadEncoder);
			motor2 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.ELEVATOR_MOTOR_2_CAN_ID,
					RobotMap.ELEVATOR_MOTOR_1_CAN_ID);
			motor3 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.ELEVATOR_MOTOR_3_CAN_ID,
					RobotMap.ELEVATOR_MOTOR_1_CAN_ID);

			configureTalons();

			motor1.setSensorPhase(false);
			motor2.setSensorPhase(false);
			motor3.setSensorPhase(false);
			motor1.setInverted(true);
			motor2.setInverted(true);
			motor3.setInverted(true);

			elevatorShift = new Solenoid(RobotMap.ELEVATOR_CLIMB_SHIFT_PCM_ID);
			frontLegShift = new Solenoid(RobotMap.FRONT_LEG_SHIFT_PCM_ID);
			backLegShift = new Solenoid(RobotMap.BACK_LEG_SHIFT_PCM_ID);

			
			// maxRevElevatorSensor = new DigitalInput(RobotMap.ELEVATOR_MAX_REV_SENSOR_DIO_ID);
			// minRevElevatorSensor = new DigitalInput(RobotMap.ELEVATOR_MIN_REV_SENSOR_DIO_ID);
		} 
		catch (Exception e) {
			System.err.println("An error occurred in the Elevator constructor");
		}
	}

	@Override
	public void initDefaultCommand() {
	}

	// Configures talons for position PID and motion magic
	public void configureTalons() {
		TalonSRXUtil.checkError(
			motor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 100),
				"Could not detect elevator encoder: ");

		TalonSRXUtil.checkError(
			motor1.configForwardSoftLimitThreshold((int)getEncoderTicks(Constants.MAX_POSITION_INCHES), Constants.kLongCANTimeoutMs),
				"Could not set forward (down) soft limit switch elevator: ");

		TalonSRXUtil.checkError(
			motor1.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs),
				"Could not enable forward (down) soft limit switch elevator: ");

		TalonSRXUtil.checkError(
			motor1.configReverseSoftLimitThreshold((int)getEncoderTicks(Constants.MIN_POSITION_INCHES), Constants.kLongCANTimeoutMs),
				"Could not set reverse (up) soft limit switch elevator: ");

		TalonSRXUtil.checkError(
			motor1.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs),
				"Could not enable reverse (up) soft limit switch elevator: ");

		TalonSRXUtil.checkError(
			motor1.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs),
				"Could not set voltage compensation saturation elevator: ");

		// configure position PID
		TalonSRXUtil.checkError(
			motor1.config_kP(kPositionPIDSlot, Constants.kElevatorPositionKp, Constants.kLongCANTimeoutMs),
				"Could not set elevator position kp: ");

		TalonSRXUtil.checkError(
			motor1.config_kI(kPositionPIDSlot, Constants.kElevatorPositionKi, Constants.kLongCANTimeoutMs),
				"Could not set elevator position ki: ");

		TalonSRXUtil.checkError(
			motor1.config_kD(kPositionPIDSlot, Constants.kElevatorPositionKd, Constants.kLongCANTimeoutMs),
				"Could not set elevator position kd: ");

		TalonSRXUtil.checkError(
			motor1.config_kF(kPositionPIDSlot, Constants.kElevatorPositionKf, Constants.kLongCANTimeoutMs),
				"Could not set elevator position kf: ");

		TalonSRXUtil.checkError(
			motor1.config_IntegralZone(kPositionPIDSlot, Constants.kElevatorIZone, Constants.kLongCANTimeoutMs),
				"Could not set elevator position i zone: ");

		TalonSRXUtil.checkError(
			motor1.configMaxIntegralAccumulator(kPositionPIDSlot,
				Constants.kElevatorMaxIntegralAccumulator, Constants.kLongCANTimeoutMs),
				"Could not set elevator position max integral: ");

		TalonSRXUtil.checkError(
			motor1.configAllowableClosedloopError(kPositionPIDSlot, Constants.kElevatorDeadband,
				Constants.kLongCANTimeoutMs), "Could not set elevator position deadband: ");

		// configure magic motion
		TalonSRXUtil.checkError(
			motor1.config_kP(kMotionMagicSlot, Constants.kElevatorMotionMagicKp, Constants.kLongCANTimeoutMs),
				"Could not set elevator motion magic kp: ");

		TalonSRXUtil.checkError(
			motor1.config_kI(kMotionMagicSlot, Constants.kElevatorMotionMagicKi, Constants.kLongCANTimeoutMs),
				"Could not set elevator motion magic ki: ");

		TalonSRXUtil.checkError(
			motor1.config_kD(kMotionMagicSlot, Constants.kElevatorMotionMagicKd, Constants.kLongCANTimeoutMs),
				"Could not set elevator motion magic kd: ");

		TalonSRXUtil.checkError(
			motor1.config_kF(kMotionMagicSlot, Constants.kElevatorMotionMagicKf, Constants.kLongCANTimeoutMs),
				"Could not set elevator motion magic kf: ");

		TalonSRXUtil.checkError(
			motor1.config_IntegralZone(kMotionMagicSlot, Constants.kElevatorIZone, Constants.kLongCANTimeoutMs),
				"Could not set elevator motion magic i zone: ");

		TalonSRXUtil.checkError(
			motor1.configMaxIntegralAccumulator(kMotionMagicSlot,
				Constants.kElevatorMaxIntegralAccumulator, Constants.kLongCANTimeoutMs),
				"Could not set elevator motion magic max integral: ");

		TalonSRXUtil.checkError(
			motor1.configAllowableClosedloopError(kMotionMagicSlot, Constants.kElevatorDeadband,
				Constants.kLongCANTimeoutMs), "Could not set elevator motion magic deadband: ");

		TalonSRXUtil.checkError(
			motor1.configMotionAcceleration(Constants.kElevatorAcceleration, Constants.kLongCANTimeoutMs), 
				"Could not set elevator motion magic acceleration: ");

		TalonSRXUtil.checkError(
			motor1.configMotionSCurveStrength(Constants.kSmoothing, Constants.kLongCANTimeoutMs),
				"Could not set elevator motion magic smoothing: ");

		TalonSRXUtil.checkError(
			motor1.configMotionCruiseVelocity(Constants.kElevatorCruiseVelocity, Constants.kLongCANTimeoutMs),
				"Could not set elevator motion magic cruise velocity: ");

		TalonSRXUtil.checkError(
			motor1.configNominalOutputForward(Constants.kElevatorNominalForward, Constants.kLongCANTimeoutMs),
				"Could not set nominal output forward: ");

		TalonSRXUtil.checkError(
			motor1.configNominalOutputReverse(Constants.kElevatorNominalReverse, Constants.kLongCANTimeoutMs),
				"Could not set nominal output reverse: ");

		TalonSRXUtil.checkError(
			motor1.configPeakOutputForward(Constants.kElevatorPeakForward, Constants.kLongCANTimeoutMs),
				"Could not set peak output forward: ");

		TalonSRXUtil.checkError(
			motor1.configPeakOutputReverse(Constants.kElevatorPeakReverse, Constants.kLongCANTimeoutMs),
				"Could not set peak output reverse: ");
		
		TalonSRXUtil.checkError(
			motor1.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, Constants.kLongCANTimeoutMs),
				"Could not set peak output reverse: ");

		TalonSRXUtil.checkError(
			motor1.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kLongCANTimeoutMs),
				"Could not set peak output reverse: ");

		TalonSRXUtil.checkError(
			motor1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets , 10, Constants.kLongCANTimeoutMs),
				"Could not set peak output reverse: ");

		motor1.enableVoltageCompensation(true);
		motor1.selectProfileSlot(kMotionMagicSlot, 0);
		
		// motor1.overrideLimitSwitchesEnable(true);
		// motor1.overrideSoftLimitsEnable(true);
		// motor1.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, 0);
		// motor1.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, 0);
	}

	public synchronized void resetEncoders() {
		motor1.setPosition(0);
	}

	private synchronized void setElevatorControlMode(ElevatorControlMode controlMode) {
		this.elevatorControlMode = controlMode;
	}

	private synchronized ElevatorControlMode getElevatorControlMode() {
		return this.elevatorControlMode;
	}

	public void setSpeed(double speed) {
		setElevatorControlMode(ElevatorControlMode.MANUAL);
		motor1.set(ControlMode.PercentOutput, speed);
	}

	public void setJoystickOpenLoop() {
		setElevatorControlMode(ElevatorControlMode.JOYSTICK_MANUAL);
	}

	public void setJoystickPID() {
		if (getElevatorControlMode() != ElevatorControlMode.JOYSTICK_POSITION_PID) {
			setElevatorControlMode(ElevatorControlMode.JOYSTICK_POSITION_PID);
			motor1.selectProfileSlot(kPositionPIDSlot, 0);
		}
		targetPositionTicks = motor1.getSelectedSensorPosition();
	}

	public synchronized void setMotionMagicPosition(double positionInchesOffGround) {
		if (getElevatorControlMode() != ElevatorControlMode.MOTION_MAGIC) {
			setElevatorControlMode(ElevatorControlMode.MOTION_MAGIC);
			motor1.selectProfileSlot(kMotionMagicSlot, 0);
		}
		targetPositionTicks = getEncoderTicks(positionInchesOffGround);
		motor1.set(ControlMode.MotionMagic, targetPositionTicks, DemandType.ArbitraryFeedForward, Constants.kElevatorFeedforwardNoBall);
	}

	private int getEncoderTicks(double positionInchesOffGround) {
		double positionInchesFromHome = positionInchesOffGround - Constants.HOME_POSITION_INCHES;
		return (int)(positionInchesFromHome * currentInchesToEncoderTicks);
	}

	public synchronized boolean hasFinishedTrajectory() {
		return elevatorControlMode == ElevatorControlMode.MOTION_MAGIC
				&& Util.epsilonEquals(motor1.getActiveTrajectoryPosition(), targetPositionTicks, 5);
	}

	private double limitPosition(double targetPosition) {
		if (targetPosition < Constants.MIN_POSITION_INCHES) {
			return Constants.MIN_POSITION_INCHES;
		} else if (targetPosition > Constants.MAX_POSITION_INCHES) {
			return Constants.MAX_POSITION_INCHES;
		}

		return targetPosition;
	}

	@Override
	public void onStart(double timestamp) {
	}

	@Override
	public void onStop(double timestamp) {
	}

	@Override
	public void onLoop(double timestamp) {
		synchronized (Elevator.this) {
			switch (getElevatorControlMode()) {
			case MOTION_MAGIC:
				controlMotionMagicWithJoystick();
				break;
			case JOYSTICK_POSITION_PID:
				controlPidWithJoystick();
				break;
			case JOYSTICK_MANUAL:
				controlManualWithJoystick();
				break;
			case MANUAL:
				break;
			default:
				System.out.println("Unknown elevator control mode");
				break;
			}
		}
	}

	private void controlMotionMagicWithJoystick() {
		double joystickPosition = -Robot.oi.getOperatorController().getLeftYAxis();
		if (Math.abs(joystickPosition) > 0.05) {
			double deltaPosition = joystickPosition * joystickTicksPerMs;
			targetPositionTicks = targetPositionTicks + deltaPosition;
			motor1.set(ControlMode.MotionMagic, targetPositionTicks, DemandType.ArbitraryFeedForward, Constants.kElevatorFeedforwardNoBall);
		}
	}

	private void controlPidWithJoystick() {
		double joystickPosition = -Robot.oi.getOperatorController().getLeftYAxis();
		double deltaPosition = joystickPosition * joystickTicksPerMs;
		targetPositionTicks = targetPositionTicks + deltaPosition;
		motor1.set(ControlMode.Position, targetPositionTicks, DemandType.ArbitraryFeedForward, Constants.kElevatorFeedforwardNoBall);
	}

	private void controlManualWithJoystick() {
		joyStickSpeed = 0.3 * -Robot.oi.getOperatorController().getLeftYAxis();
		motor1.set(ControlMode.PercentOutput, joyStickSpeed);
	}

	public synchronized double getRPM() {
		// We are using a CTRE mag encoder which is 4096 native units per revolution.
		double velocity_ticks_per_100ms = motor1.getSelectedSensorVelocity(0);
		return velocity_ticks_per_100ms * 10.0 / 4096.0 * 60.0;
	}

	public synchronized double getInchesOffGround() {
		double position_ticks = motor1.getSelectedSensorPosition(0);
		return (position_ticks / currentInchesToEncoderTicks) + Constants.HOME_POSITION_INCHES;
	}

	public synchronized double getSetpointInches() {
		return elevatorControlMode == ElevatorControlMode.MOTION_MAGIC
				? targetPositionTicks / currentInchesToEncoderTicks + Constants.HOME_POSITION_INCHES
				: Double.NaN;
	}

	public void setFrontLegState(FrontLegShiftState state) {
		frontShiftState = state;
		if (state == FrontLegShiftState.LOCKED) {
			frontLegShift.set(false);
			System.out.println("FL IN NOT CLIMB");
		} else if (state == FrontLegShiftState.ENGAGED) {
			frontLegShift.set(true);
			System.out.println("FL OUT CLIMB MODE");
		}
	}

	public void setBackLegState(BackLegShiftState state) {
		backShiftState = state;
		if (state == BackLegShiftState.LOCKED) {
			backLegShift.set(false);
			System.out.println("BL IN NOT CLIMB");
		} else if (state == BackLegShiftState.ENGAGED) {
			backLegShift.set(true);
			System.out.println("BL OUT CLIMB MODE");
		}
	}

	public void setElevatorClimbState(ElevatorClimbShiftState state) {
		elevatorClimbShiftState = state;
		if (state == ElevatorClimbShiftState.LOCKED) {
			elevatorShift.set(false);
			System.out.println("GG IN CLIMB");
		} else if (state == ElevatorClimbShiftState.ENGAGED) {
			elevatorShift.set(true);
			System.out.println("GG OUT NOT ClIMb");
		}
	}

	public void setRobotClimbMode() {
		joystickTicksPerMs = JOYSTICK_TICKS_PER_MS_GGG;
		currentInchesToEncoderTicks = INCHES_TO_ENCODER_TICKS_GGG;
		setFrontLegState(FrontLegShiftState.ENGAGED);
		setBackLegState(BackLegShiftState.ENGAGED);
		setElevatorClimbState(ElevatorClimbShiftState.LOCKED);
	}

	public void setRobotClimbFront() {
		joystickTicksPerMs = JOYSTICK_TICKS_PER_MS_GGG;
		currentInchesToEncoderTicks = INCHES_TO_ENCODER_TICKS_GGG;
		setFrontLegState(FrontLegShiftState.ENGAGED);
		setBackLegState(BackLegShiftState.LOCKED);
		setElevatorClimbState(ElevatorClimbShiftState.LOCKED);
	}

	public void setRobotClimbBack() {
		joystickTicksPerMs = JOYSTICK_TICKS_PER_MS_GGG;
		currentInchesToEncoderTicks = INCHES_TO_ENCODER_TICKS_GGG;
		setFrontLegState(FrontLegShiftState.LOCKED);
		setBackLegState(BackLegShiftState.ENGAGED);
		setElevatorClimbState(ElevatorClimbShiftState.LOCKED);
	}

	public void setRobotScoreMode() {
		joystickTicksPerMs = JOYSTICK_TICKS_PER_MS_ELEVATOR;
		currentInchesToEncoderTicks = INCHES_TO_ENCODER_TICKS_ELEVATOR;
		setFrontLegState(FrontLegShiftState.LOCKED);
		setBackLegState(BackLegShiftState.LOCKED);
		setElevatorClimbState(ElevatorClimbShiftState.ENGAGED);
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

	public double getAverageMotorCurrent() {
		return (motor1.getOutputCurrent() + motor2.getOutputCurrent() + motor3.getOutputCurrent()) / 3;
	}

	// public boolean getMaxElevatorSensor() {
	// 	return maxRevElevatorSensor.get();
	// }

	// public boolean getMinElevatorSensor() {
	// 	return minRevElevatorSensor.get();
	// }

	public void updateStatus(Robot.OperationMode operationMode) {
		if (operationMode == Robot.OperationMode.TEST) {
			try {
				SmartDashboard.putNumber("Elevator Position Inches", motor1.getPositionWorld());
				SmartDashboard.putNumber("Elevator Motor 1 Amps", motor1.getOutputCurrent());
				SmartDashboard.putNumber("Elevator Motor 2 Amps", motor2.getOutputCurrent());
				SmartDashboard.putNumber("Elevator Motor 3 Amps", motor3.getOutputCurrent());
				SmartDashboard.putNumber("Elevator Average Amps", getAverageMotorCurrent());
				SmartDashboard.putNumber("Elevator Target Position Ticks", targetPositionTicks);
				SmartDashboard.putNumber("Elevator Position Inches", getInchesOffGround());
				SmartDashboard.putNumber("ActTrajVelocity", motor1.getActiveTrajectoryVelocity());
				SmartDashboard.putNumber("ActTrajPosition", motor1.getActiveTrajectoryPosition());
				SmartDashboard.putNumber("SensorVel", motor1.getSelectedSensorVelocity());
				SmartDashboard.putNumber("SensorPos", motor1.getSelectedSensorPosition());
				SmartDashboard.putNumber("MotorOutputPercent", motor1.getMotorOutputPercent());
				SmartDashboard.putNumber("ClosedLoopError", motor1.getClosedLoopError());
				SmartDashboard.putNumber("Elevator Joystick", joyStickSpeed);
			} catch (Exception e) {
			}
		} else if (operationMode == Robot.OperationMode.COMPETITION) {
			// SmartDashboard.putBoolean("Elevator Max Switch = ", getMaxElevatorSensor());
			// SmartDashboard.putBoolean("Elevator Min Switch = ", getMinElevatorSensor());

		}
	}

	public static Elevator getInstance() {
		if (instance == null) {
			instance = new Elevator();
		}
		return instance;
	}
}