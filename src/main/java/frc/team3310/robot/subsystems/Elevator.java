package frc.team3310.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
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

	public static final double INCHES_TO_ENCODER_TICKS_ELEVATOR = (50.0 / 50.0) * (34.0 / 34.0) * 4096.0
			/ (1.2987013 * Math.PI);

																							// 24.0 / 40.0
	public static final double INCHES_TO_ENCODER_TICKS_GGG = (50.0 / 50.0) * (50.0 / 18.0) * (30.0 / 34.0) * 4096.0
			/ (1.077 * Math.PI);

	// Defined speeds
	public static final double TEST_SPEED_UP = 0.3;
	public static final double TEST_SPEED_DOWN = -0.2;
	public static final double AUTO_ZERO_SPEED = -0.25;
	public static final double JOYSTICK_TICKS_PER_MS_ELEVATOR = 0.5 * INCHES_TO_ENCODER_TICKS_ELEVATOR;
	public static final double JOYSTICK_TICKS_PER_MS_GGG = JOYSTICK_TICKS_PER_MS_ELEVATOR
			/ INCHES_TO_ENCODER_TICKS_ELEVATOR * INCHES_TO_ENCODER_TICKS_GGG * 0.8;

	// Motor controllers
	private TalonSRXEncoder motor1;
	private TalonSRX motor2;
	private TalonSRX motor3;

	// Sensors
	private DigitalInput maxRevElevatorSensor;
	private DigitalInput minRevElevatorSensor;
	private DigitalInput climbRearTop;
	private DigitalInput climbRearBot;
	private DigitalInput climbFrontTop;
	private DigitalInput climbFrontBot;
	private DigitalInput platfromDetectFront;
	private DigitalInput platformDetectRear;

	// Pneumatics
	private Solenoid elevatorShift;
	private Solenoid frontLegShift;
	private Solenoid backLegShift;

	// Position PID
	private static final int kPositionPIDSlot = 0;

	// Motion Magic
	private static final int kElevatorMotionMagicSlot = 1;
	private static final int kClimbMotionMagicSlot = 2;

	// Shifting Default
	private FrontLegShiftState frontShiftState = FrontLegShiftState.LOCKED; // ENGAGED = Climb Mode
	private BackLegShiftState backShiftState = BackLegShiftState.LOCKED; // ENGAGED = Climb Mode
	private ElevatorClimbShiftState elevatorClimbShiftState = ElevatorClimbShiftState.ENGAGED; // LOCKED = Climb Mode

	// Misc
	public static final double AUTO_ZERO_MOTOR_CURRENT = 0.8;
	private double joystickTicksPerMs = JOYSTICK_TICKS_PER_MS_ELEVATOR;
	private double targetPositionTicks = 0;
	private double joyStickSpeed;
	private double homePosition = Constants.AUTO_HOME_POSITION_INCHES;
	public boolean elevatorCargoMode = false;
	public boolean overrideClimb = false;

	private ElevatorControlMode elevatorControlMode = ElevatorControlMode.MOTION_MAGIC;

	// Constructor
	private Elevator() {
		try {
			motor1 = TalonSRXFactory.createTalonEncoder(RobotMap.ELEVATOR_MOTOR_1_CAN_ID,
					INCHES_TO_ENCODER_TICKS_ELEVATOR, false, FeedbackDevice.QuadEncoder);
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

			maxRevElevatorSensor = new DigitalInput(RobotMap.ELEVATOR_MAX_REV_SENSOR_DIO_ID);
			minRevElevatorSensor = new DigitalInput(RobotMap.ELEVATOR_MIN_REV_SENSOR_DIO_ID);
			climbFrontTop = new DigitalInput(RobotMap.CLIMB_FRONT_TOP_SENSOR_DIO_ID);
			climbFrontBot = new DigitalInput(RobotMap.CLIMB_FRONT_BOT_SENSOR_DIO_ID);
			climbRearTop = new DigitalInput(RobotMap.CLIMB_REAR_TOP_SENSOR_DIO_ID);
			climbRearBot = new DigitalInput(RobotMap.CLIMB_REAR_BOT_SENSOR_DIO_ID);
			platfromDetectFront = new DigitalInput(RobotMap.PLATFORM_DETECT_FRONT_SENSOR_DIO_ID);
			platformDetectRear = new DigitalInput(RobotMap.PLATFORM_DETECT_REAR_SENSROR_DIO_ID);

			elevatorShift = new Solenoid(RobotMap.ELEVATOR_CLIMB_SHIFT_PCM_ID);
			frontLegShift = new Solenoid(RobotMap.FRONT_LEG_SHIFT_PCM_ID);
			backLegShift = new Solenoid(RobotMap.BACK_LEG_SHIFT_PCM_ID);

		} catch (Exception e) {
			System.err.println("An error occurred in the Elevator constructor");
		}
	}

	@Override
	public void initDefaultCommand() {
	}

	// Configures talons for position PID and motion magic
	public void configureTalons() {
		TalonSRXUtil.checkError(motor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 100),
				"Could not detect elevator encoder: ");

		TalonSRXUtil.checkError(
				motor1.configForwardSoftLimitThreshold((int) getElevatorEncoderTicks(Constants.MAX_POSITION_INCHES),
						Constants.kLongCANTimeoutMs),
				"Could not set forward (down) soft limit switch elevator: ");

		TalonSRXUtil.checkError(motor1.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs),
				"Could not enable forward (down) soft limit switch elevator: ");

		TalonSRXUtil.checkError(
				motor1.configReverseSoftLimitThreshold((int) getElevatorEncoderTicks(Constants.MIN_POSITION_INCHES),
						Constants.kLongCANTimeoutMs),
				"Could not set reverse (up) soft limit switch elevator: ");

		TalonSRXUtil.checkError(motor1.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs),
				"Could not enable reverse (up) soft limit switch elevator: ");

		TalonSRXUtil.checkError(motor1.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs),
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

		TalonSRXUtil.checkError(motor1.configMaxIntegralAccumulator(kPositionPIDSlot,
				Constants.kElevatorMaxIntegralAccumulator, Constants.kLongCANTimeoutMs),
				"Could not set elevator position max integral: ");

		TalonSRXUtil.checkError(motor1.configAllowableClosedloopError(kPositionPIDSlot, Constants.kElevatorDeadband,
				Constants.kLongCANTimeoutMs), "Could not set elevator position deadband: ");

		// Configure magic motion elevator
		TalonSRXUtil.checkError(motor1.config_kP(kElevatorMotionMagicSlot, Constants.kElevatorMotionMagicKp,
				Constants.kLongCANTimeoutMs), "Could not set elevator motion magic kp: ");

		TalonSRXUtil.checkError(motor1.config_kI(kElevatorMotionMagicSlot, Constants.kElevatorMotionMagicKi,
				Constants.kLongCANTimeoutMs), "Could not set elevator motion magic ki: ");

		TalonSRXUtil.checkError(motor1.config_kD(kElevatorMotionMagicSlot, Constants.kElevatorMotionMagicKd,
				Constants.kLongCANTimeoutMs), "Could not set elevator motion magic kd: ");

		TalonSRXUtil.checkError(motor1.config_kF(kElevatorMotionMagicSlot, Constants.kElevatorMotionMagicKf,
				Constants.kLongCANTimeoutMs), "Could not set elevator motion magic kf: ");

		TalonSRXUtil.checkError(motor1.config_IntegralZone(kElevatorMotionMagicSlot, Constants.kElevatorIZone,
				Constants.kLongCANTimeoutMs), "Could not set elevator motion magic i zone: ");

		TalonSRXUtil
				.checkError(
						motor1.configMaxIntegralAccumulator(kElevatorMotionMagicSlot,
								Constants.kElevatorMaxIntegralAccumulator, Constants.kLongCANTimeoutMs),
						"Could not set elevator motion magic max integral: ");

		TalonSRXUtil.checkError(motor1.configAllowableClosedloopError(kElevatorMotionMagicSlot,
				Constants.kElevatorDeadband, Constants.kLongCANTimeoutMs),
				"Could not set elevator motion magic deadband: ");

		TalonSRXUtil.checkError(
				motor1.config_kP(kClimbMotionMagicSlot, Constants.kElevatorMotionMagicKp, Constants.kLongCANTimeoutMs),
				"Could not set elevator motion magic kp: ");

		TalonSRXUtil.checkError(
				motor1.config_kI(kClimbMotionMagicSlot, Constants.kElevatorMotionMagicKi, Constants.kLongCANTimeoutMs),
				"Could not set elevator motion magic ki: ");

		TalonSRXUtil.checkError(
				motor1.config_kD(kClimbMotionMagicSlot, Constants.kElevatorMotionMagicKd, Constants.kLongCANTimeoutMs),
				"Could not set elevator motion magic kd: ");

		TalonSRXUtil.checkError(
				motor1.config_kF(kClimbMotionMagicSlot, Constants.kElevatorMotionMagicKf, Constants.kLongCANTimeoutMs),
				"Could not set elevator motion magic kf: ");

		TalonSRXUtil.checkError(motor1.config_IntegralZone(kClimbMotionMagicSlot, Constants.kElevatorIZone,
				Constants.kLongCANTimeoutMs), "Could not set elevator motion magic i zone: ");

		// Motion Magic Climb
		TalonSRXUtil
				.checkError(
						motor1.configMaxIntegralAccumulator(kClimbMotionMagicSlot,
								Constants.kElevatorMaxIntegralAccumulator, Constants.kLongCANTimeoutMs),
						"Could not set elevator motion magic max integral: ");

		TalonSRXUtil.checkError(motor1.configAllowableClosedloopError(kClimbMotionMagicSlot,
				Constants.kElevatorDeadband, Constants.kLongCANTimeoutMs),
				"Could not set elevator motion magic deadband: ");

		TalonSRXUtil.checkError(
				motor1.configMotionAcceleration(Constants.kElevatorAcceleration, Constants.kLongCANTimeoutMs),
				"Could not set elevator motion magic acceleration: ");

		TalonSRXUtil.checkError(
				motor1.configMotionSCurveStrength(Constants.kElevatorScurveStrength, Constants.kLongCANTimeoutMs),
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
				motor1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, Constants.kLongCANTimeoutMs),
				"Could not set peak output reverse: ");

		motor1.enableVoltageCompensation(true);
		motor1.selectProfileSlot(kElevatorMotionMagicSlot, 0);

		// motor1.overrideLimitSwitchesEnable(true);
		// motor1.overrideSoftLimitsEnable(true);
		// motor1.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, 0);
		// motor1.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, 0);
	}

	public synchronized void resetEncoders() {
		motor1.setPosition(0);
	}

	public synchronized void resetEncoders(double homePosition) {
		motor1.setPosition(0);
		this.homePosition = homePosition;
	}

	public boolean isClimbOverrided() {
		return overrideClimb;
	}

	public boolean getMaxElevatorSensor() {
		return !maxRevElevatorSensor.get();
	}

	public boolean getMinElevatorSensor() {
		return !minRevElevatorSensor.get();
	}

	public boolean getClimbFrontTop() {
		return !climbFrontTop.get();
	}

	public boolean getClimbFrontBot() {
		return climbFrontBot.get();
	}

	public boolean getClimbRearTop() {
		return !climbRearTop.get();
	}

	public boolean getClimbRearBot() {
		return climbRearBot.get();
	}

	public boolean getPlatformDetectFront() {
		return platfromDetectFront.get();
	}

	public boolean getPlatformDetectRear() {
		return platformDetectRear.get();
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

	public synchronized void setElevatorMotionMagicPosition(double positionInchesOffGround) {
		if (getElevatorControlMode() != ElevatorControlMode.MOTION_MAGIC) {
			setElevatorControlMode(ElevatorControlMode.MOTION_MAGIC);
		}
		motor1.selectProfileSlot(kElevatorMotionMagicSlot, 0);
		targetPositionTicks = getElevatorEncoderTicks(positionInchesOffGround);
		motor1.set(ControlMode.MotionMagic, targetPositionTicks, DemandType.ArbitraryFeedForward,
				Constants.kElevatorFeedforwardNoBall);
	}

	public synchronized void setClimbMotionMagicPosition(double positionInches) {
		if (getElevatorControlMode() != ElevatorControlMode.MOTION_MAGIC) {
			setElevatorControlMode(ElevatorControlMode.MOTION_MAGIC);
		}
				motor1.configMotionAcceleration(Constants.kClimbAcceleration, 0);

				motor1.configMotionSCurveStrength(Constants.kClimbScurveStrength, 0);
	
				motor1.configMotionCruiseVelocity(Constants.kClimbCruiseVelocity, 0);
		motor1.selectProfileSlot(kClimbMotionMagicSlot, 0);
		targetPositionTicks = getClimbEncoderTicks(positionInches);
		motor1.set(ControlMode.MotionMagic, targetPositionTicks, DemandType.ArbitraryFeedForward,
				Constants.kElevatorFeedforwardNoBall);
	}

	private int getElevatorEncoderTicks(double positionInchesOffGround) {
		double positionInchesFromHome = positionInchesOffGround - homePosition;
		return (int) (positionInchesFromHome * INCHES_TO_ENCODER_TICKS_ELEVATOR);
	}

	private int getClimbEncoderTicks(double positionInches) {
		return (int) (positionInches * INCHES_TO_ENCODER_TICKS_GGG);
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
				// controlMotionMagicWithJoystick();
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

	// private void controlMotionMagicWithJoystick() {
	// double joystickPosition = -Robot.oi.getOperatorController().getLeftYAxis();
	// if (Math.abs(joystickPosition) > 0.05) {
	// double deltaPosition = joystickPosition * joystickTicksPerMs;
	// targetPositionTicks = targetPositionTicks + deltaPosition;
	// motor1.set(ControlMode.MotionMagic, targetPositionTicks,
	// DemandType.ArbitraryFeedForward,
	// Constants.kElevatorFeedforwardNoBall);
	// }
	// }

	private void controlPidWithJoystick() {
		double joystickPosition = -Robot.oi.getOperatorController().getLeftYAxis();
		if (joystickPosition > 0.04 || joystickPosition < -0.04) {
			double deltaPosition = joystickPosition * joystickTicksPerMs;
			targetPositionTicks = targetPositionTicks + deltaPosition;
			motor1.set(ControlMode.Position, targetPositionTicks, DemandType.ArbitraryFeedForward,
					Constants.kElevatorFeedforwardNoBall);
		}
	}

	private void controlManualWithJoystick() {
		joyStickSpeed = 1.0 * -Robot.oi.getOperatorController().getLeftYAxis();
		motor1.set(ControlMode.PercentOutput, joyStickSpeed);
	}

	public synchronized double getRPM() {
		// We are using a CTRE mag encoder which is 4096 native units per revolution.
		double velocity_ticks_per_100ms = motor1.getSelectedSensorVelocity(0);
		return velocity_ticks_per_100ms * 10.0 / 4096.0 * 60.0;
	}

	public synchronized double getElevatorInchesOffGround() {
		double position_ticks = motor1.getSelectedSensorPosition(0);
		return (position_ticks / INCHES_TO_ENCODER_TICKS_ELEVATOR) + homePosition;
	}

	public synchronized double getElevatorSetpointInches() {
		return elevatorControlMode == ElevatorControlMode.MOTION_MAGIC
				? targetPositionTicks / INCHES_TO_ENCODER_TICKS_ELEVATOR + homePosition
				: Double.NaN;
	}

	public void setFrontLegState(FrontLegShiftState state) {
		frontShiftState = state;
		if (state == FrontLegShiftState.LOCKED) {
			frontLegShift.set(false);
			// System.out.println("FL IN NOT CLIMB");
		} else if (state == FrontLegShiftState.ENGAGED) {
			frontLegShift.set(true);
			// System.out.println("FL OUT CLIMB MODE");
		}
	}

	public void setBackLegState(BackLegShiftState state) {
		backShiftState = state;
		if (state == BackLegShiftState.LOCKED) {
			backLegShift.set(false);
			// System.out.println("BL IN NOT CLIMB");
		} else if (state == BackLegShiftState.ENGAGED) {
			backLegShift.set(true);
			// System.out.println("BL OUT CLIMB MODE");
		}
	}

	public void setElevatorClimbState(ElevatorClimbShiftState state) {
		elevatorClimbShiftState = state;
		if (state == ElevatorClimbShiftState.LOCKED) {
			elevatorShift.set(false);
			// System.out.println("GG IN CLIMB");
		} else if (state == ElevatorClimbShiftState.ENGAGED) {
			elevatorShift.set(true);
			// System.out.println("GG OUT NOT CLIMB");
		}
	}

	public void setRobotClimbMode() {
		joystickTicksPerMs = JOYSTICK_TICKS_PER_MS_GGG;
		setFrontLegState(FrontLegShiftState.ENGAGED);
		setBackLegState(BackLegShiftState.ENGAGED);
		setElevatorClimbState(ElevatorClimbShiftState.LOCKED);
	}

	public void setRobotClimbFront() {
		joystickTicksPerMs = JOYSTICK_TICKS_PER_MS_GGG;
		setFrontLegState(FrontLegShiftState.ENGAGED);
		setBackLegState(BackLegShiftState.LOCKED);
		setElevatorClimbState(ElevatorClimbShiftState.LOCKED);
	}

	public void setRobotClimbBack() {
		joystickTicksPerMs = JOYSTICK_TICKS_PER_MS_GGG;
		setFrontLegState(FrontLegShiftState.LOCKED);
		setBackLegState(BackLegShiftState.ENGAGED);
		setElevatorClimbState(ElevatorClimbShiftState.LOCKED);
	}

	public void setRobotScoreMode() {
		joystickTicksPerMs = JOYSTICK_TICKS_PER_MS_ELEVATOR;
		setFrontLegState(FrontLegShiftState.LOCKED);
		setBackLegState(BackLegShiftState.LOCKED);
		setElevatorClimbState(ElevatorClimbShiftState.ENGAGED);
	}

	public void setRobotLockedMode() {
		joystickTicksPerMs = JOYSTICK_TICKS_PER_MS_ELEVATOR;
		setFrontLegState(FrontLegShiftState.LOCKED);
		setBackLegState(BackLegShiftState.LOCKED);
		setElevatorClimbState(ElevatorClimbShiftState.LOCKED);
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

	public double getElevatorPositionInches() {
		return motor1.getPositionWorld();
	}

	public double getClimbPositionInches() {
		double position_ticks = motor1.getSelectedSensorPosition(0);
		return position_ticks / INCHES_TO_ENCODER_TICKS_GGG;
	}

	public double getAverageMotorCurrent() {
		return (motor1.getOutputCurrent() + motor2.getOutputCurrent() + motor3.getOutputCurrent()) / 3;
	}

	public void updateStatus(Robot.OperationMode operationMode) {
		if (operationMode == Robot.OperationMode.TEST) {
			try {
				SmartDashboard.putNumber("Elevator Position Inches", motor1.getPositionWorld());
				SmartDashboard.putNumber("Elevator Motor 1 Amps", motor1.getOutputCurrent());
				SmartDashboard.putNumber("Elevator Motor 2 Amps", motor2.getOutputCurrent());
				SmartDashboard.putNumber("Elevator Motor 3 Amps", motor3.getOutputCurrent());
				SmartDashboard.putNumber("Elevator Average Amps", getAverageMotorCurrent());
				SmartDashboard.putNumber("Elevator Target Position Ticks", targetPositionTicks);
				SmartDashboard.putNumber("Elevator Position Inches", getElevatorInchesOffGround());
				SmartDashboard.putNumber("ActTrajVelocity", motor1.getActiveTrajectoryVelocity());
				SmartDashboard.putNumber("ActTrajPosition", motor1.getActiveTrajectoryPosition());
				SmartDashboard.putNumber("SensorVel", motor1.getSelectedSensorVelocity());
				SmartDashboard.putNumber("SensorPos", motor1.getSelectedSensorPosition());
				SmartDashboard.putNumber("MotorOutputPercent", motor1.getMotorOutputPercent());
				SmartDashboard.putNumber("ClosedLoopError", motor1.getClosedLoopError());
				SmartDashboard.putBoolean("Climb Front Top = ", getClimbFrontTop());
				SmartDashboard.putBoolean("Climb Front Bot = ", getClimbFrontBot());
				SmartDashboard.putBoolean("Climb Rear Top = ", getClimbFrontTop());
				SmartDashboard.putBoolean("Climb Rear Bot = ", getClimbRearBot());
				SmartDashboard.putNumber("Climb Position Inches", getClimbPositionInches());
				SmartDashboard.putBoolean("Elevator Max Switch = ", getMaxElevatorSensor());
				SmartDashboard.putBoolean("Elevator Min Switch = ", getMinElevatorSensor());
				SmartDashboard.putBoolean("Platform Detect Front = ", getPlatformDetectFront());
				SmartDashboard.putBoolean("Plaform Detect Bot = ", getPlatformDetectRear());
				SmartDashboard.putNumber("Elevator Joystick", joyStickSpeed);
			} catch (Exception e) {
			}
		} else if (operationMode == Robot.OperationMode.COMPETITION) {
			SmartDashboard.putBoolean("Elevator Max Switch = ", getMaxElevatorSensor());
			SmartDashboard.putBoolean("Elevator Min Switch = ", getMinElevatorSensor());
			SmartDashboard.putBoolean("Platform Detect Front = ", getPlatformDetectFront());
			SmartDashboard.putBoolean("Plaform Detect Bot = ", getPlatformDetectRear());
			SmartDashboard.putNumber("Climb Position Inches", getClimbPositionInches());
			



		}
	}

	public static Elevator getInstance() {
		if (instance == null) {
			instance = new Elevator();
		}
		return instance;
	}
}